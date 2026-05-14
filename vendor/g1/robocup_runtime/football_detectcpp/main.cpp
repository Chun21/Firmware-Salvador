#include <sys/stat.h> //文件状态检查和文件操作
#include <unistd.h> //进程控制、文件操作、系统调用
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream> //C++标准输入/输出流库
#include <sstream>
#include <string> //C++标准字符串库
#include <vector>
#include "YOLO.h" //YOLO目标检测类头文件
#include "detection_filter.h"
#include "RgbdLocalizer.h"

#include <thread> // 标准线程库
#include <atomic> //用于 统计检测帧数 或 控制线程同步（如 std::atomic<bool> 控制线程退出）。
#include <queue> //标准库队列，通常用于 任务缓冲（生产者-消费者模型）
#include <mutex> //互斥锁，保护任务队列（如多线程推入/取出检测任务）
#include "dds/DetectionModule.hpp" //自定义 DDS 检测模块
#include "common/LocationModule.hpp"
#include <dds/dds.hpp> //DDS核心库，发布检测结果
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <memory>
#include "../robocup_locator_v1.1/include/dds/Subscription.h"
#include"common.h" //项目公共头文件
/**
 * @brief Setting up Tensorrt logger //TensorRT日志记录器设置
*/
class Logger : public nvinfer1::ILogger {//继承TensorRT的日志接口
    void log(Severity severity, const char* msg) noexcept override {
        // 只输出严重级别高于警告的日志
        if (severity <= Severity::kWARNING)
            std::cout << msg << std::endl;// 将日志消息（msg）输出，并在末尾添加换行符（std::endl）
    }
}logger;

int read_env_int(const char* key, int default_value) {
    const char* raw = std::getenv(key);
    if (raw == nullptr || raw[0] == '\0') {
        return default_value;
    }
    char* end = nullptr;
    const long parsed = std::strtol(raw, &end, 10);
    if (end == raw || parsed <= 0) {
        std::cerr << "Invalid integer env " << key << "=" << raw
                  << ", using default " << default_value << std::endl;
        return default_value;
    }
    return static_cast<int>(parsed);
}

float read_env_float(const char* key, float default_value) {
    const char* raw = std::getenv(key);
    if (raw == nullptr || raw[0] == '\0') {
        return default_value;
    }
    char* end = nullptr;
    const float parsed = std::strtof(raw, &end);
    if (end == raw || !std::isfinite(parsed)) {
        std::cerr << "Invalid float env " << key << "=" << raw
                  << ", using default " << default_value << std::endl;
        return default_value;
    }
    return parsed;
}

// 图像参数定义。默认使用 640x480@30，保证识别帧率；仍可通过环境变量临时覆盖。
int image_width = read_env_int("ROBOCUP_CAMERA_WIDTH", 640);//图像横向分辨率
int image_height = read_env_int("ROBOCUP_CAMERA_HEIGHT", 480); //图像纵向分辨率

float init_fps = read_env_int("ROBOCUP_CAMERA_FPS", 30);// 初始帧率(FPS)
// Gemini 336L color camera FoV from the vendor spec sheet.
float horizontal_fov = 89;// 水平视场角度
float vertical_fov = 64;  // 垂直视场角度
// float horizontal_fov = 86;// 水平视场角度
// float vertical_fov = 57;  // 垂直视场角度

constexpr const char* kPreferredD455SerialEnv = "ROBOCUP_D455_SERIAL";
constexpr const char* kNetworkInterfaceEnv = "ROBOCUP_NET_IFACE";
constexpr const char* kRgbdLocationTopicEnv = "ROBOCUP_RGBD_LOCATION_TOPIC";
constexpr const char* kDefaultRgbdLocationTopic = "rt/locationresults_rgbd";
constexpr std::size_t kHeadYawIndex = 0;
constexpr std::size_t kHeadPitchIndex = 1;
constexpr std::size_t kWaistYawIndex = 12;

std::pair<float, float> safe_calculate_angles(
    float dx,
    float dy,
    int image_width,
    int image_height,
    float horizontal_fov,
    float vertical_fov);

std::string get_env_or_empty(const char* key) {
    const char* value = std::getenv(key);
    return value == nullptr ? std::string() : std::string(value);
}

std::string to_lower_copy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

std::string get_camera_info(const rs2::device& device, rs2_camera_info info) {
    if (!device.supports(info)) {
        return {};
    }
    return device.get_info(info);
}

bool is_d455_device(const rs2::device& device) {
    return to_lower_copy(get_camera_info(device, RS2_CAMERA_INFO_NAME)).find("d455") != std::string::npos;
}

std::pair<float, float> calculate_angles_from_intrinsics_or_fov(
    float u,
    float v,
    int image_width,
    int image_height,
    const rs2_intrinsics& intrinsics)
{
    if (intrinsics.fx > 1e-6f && intrinsics.fy > 1e-6f) {
        const float yaw = std::atan2(u - intrinsics.ppx, intrinsics.fx) * 180.0f / static_cast<float>(M_PI);
        const float pitch = -std::atan2(v - intrinsics.ppy, intrinsics.fy) * 180.0f / static_cast<float>(M_PI);
        return {yaw, pitch};
    }

    const float offset_x = u - image_width / 2.0f;
    const float offset_y = v - image_height / 2.0f;
    return safe_calculate_angles(offset_x, offset_y, image_width, image_height, horizontal_fov, vertical_fov);
}

// Camera reconnection logic相机控制器类
class CameraController {
    rs2::pipeline pipe;// RealSense管道，用于管理数据流
    std::string serial;       // 相机序列号
    std::string device_name;  // 相机名称
    std::string preferred_serial;
    int color_width = image_width;
    int color_height = image_height;
    int depth_width = image_width;
    int depth_height = image_height;
    int fps = static_cast<int>(init_fps);

    struct StreamProfileRequest {
        int color_width;
        int color_height;
        int depth_width;
        int depth_height;
        int fps;
        const char* name;
    };
    
public:
    rs2_intrinsics intrinsics; // 外部访问相机内参
    float depth_scale_m = 0.001f;
 // 构造函数，配置彩色和深度流
    CameraController() : preferred_serial(get_env_or_empty(kPreferredD455SerialEnv)) {}
  // 初始化相机
    bool initialize() {
        if (!resolve_target_device()) {
            return false;
        }

        std::vector<StreamProfileRequest> candidates;
        const bool user_forced_resolution =
            std::getenv("ROBOCUP_CAMERA_WIDTH") != nullptr ||
            std::getenv("ROBOCUP_CAMERA_HEIGHT") != nullptr;

        if (user_forced_resolution) {
            const int requested_fps = read_env_int("ROBOCUP_CAMERA_FPS", static_cast<int>(init_fps));
            candidates.push_back({image_width, image_height, image_width, image_height, requested_fps, "env-forced"});
        }

        // Stable real-time default first.
        candidates.push_back({640, 480, 640, 480, 30, "640x480@30"});
        candidates.push_back({640, 480, 640, 480, 15, "640x480@15"});
        candidates.push_back({640, 480, 640, 480, 5, "640x480@5"});
        // Higher-resolution fallbacks if explicitly useful and lower modes fail.
        candidates.push_back({1280, 720, 1280, 720, 5, "1280x720@5"});
        candidates.push_back({1280, 720, 848, 480, 10, "color1280x720@10-depth848x480@10"});
        candidates.push_back({1280, 720, 848, 480, 5, "color1280x720@5-depth848x480@5"});

        std::string last_error;
        for (const auto& candidate : candidates) {
            rs2::config cfg;
            cfg.enable_device(serial);
            cfg.enable_stream(RS2_STREAM_COLOR, candidate.color_width, candidate.color_height, RS2_FORMAT_BGR8, candidate.fps);
            cfg.enable_stream(RS2_STREAM_DEPTH, candidate.depth_width, candidate.depth_height, RS2_FORMAT_Z16, candidate.fps);

            try {
                auto profile = pipe.start(cfg); // 启动管道
                auto color_stream = profile.get_stream(RS2_STREAM_COLOR)
                                    .as<rs2::video_stream_profile>(); // 获取彩色流配置
                auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH)
                                    .as<rs2::video_stream_profile>();
                intrinsics = color_stream.get_intrinsics(); // 获取相机内参
                color_width = color_stream.width();
                color_height = color_stream.height();
                depth_width = depth_stream.width();
                depth_height = depth_stream.height();
                fps = candidate.fps;
                image_width = color_width;
                image_height = color_height;

                serial = profile.get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);// 获取序列号
                device_name = get_camera_info(profile.get_device(), RS2_CAMERA_INFO_NAME);
                for (auto&& sensor : profile.get_device().query_sensors()) {
                    if (sensor.is<rs2::depth_sensor>()) {
                        depth_scale_m = sensor.as<rs2::depth_sensor>().get_depth_scale();
                        break;
                    }
                }
                std::cout << "Using RealSense camera: " << device_name
                          << " serial=" << serial
                          << " selected_profile=" << candidate.name
                          << " color=" << color_width << "x" << color_height
                          << " depth=" << depth_width << "x" << depth_height
                          << " fps=" << fps
                          << " depth_scale=" << depth_scale_m
                          << " fx=" << intrinsics.fx
                          << " fy=" << intrinsics.fy
                          << " ppx=" << intrinsics.ppx
                          << " ppy=" << intrinsics.ppy
                          << std::endl;
                return true;// 初始化成功
            } catch (const rs2::error& e) {
                last_error = e.what();
                std::cerr << "Camera profile failed: " << candidate.name
                          << " (" << last_error << "), trying fallback..." << std::endl;
            }
        }

        std::cerr << "Camera init failed: no supported RGB-D profile found. Last error: "
                  << last_error << std::endl;
        return false;// 初始化失败
    }
// 关闭相机
    void shutdown() {
        try { pipe.stop(); }// 停止管道
        catch (...) {}        // 忽略任何异常
    }
 // 轮询获取帧数据
    rs2::frameset poll_frames(int max_retries = 300) {
        for (int i = 0; i < max_retries; ++i) {
            rs2::frameset frames;
            if (pipe.poll_for_frames(&frames)) {  // 非阻塞获取帧
                return frames;  // 成功获取返回帧
            }
            std::this_thread::sleep_for(10ms); // 短暂等待
        }
        return {}; // Return empty frame to indicate failure// 失败返回空帧
    }
 // 获取相机序列号
    std::string get_serial() const { return serial; }
    std::string get_device_name() const { return device_name; }
    int get_color_width() const { return color_width; }
    int get_color_height() const { return color_height; }

private:
    bool resolve_target_device() {
        rs2::context ctx;
        auto devices = ctx.query_devices();
        if (devices.size() == 0) {
            std::cerr << "No RealSense camera detected" << std::endl;
            return false;
        }

        std::vector<std::pair<std::string, std::string>> d455_devices;
        std::vector<std::pair<std::string, std::string>> realsense_devices;
        for (auto&& device : devices) {
            const std::string device_serial = get_camera_info(device, RS2_CAMERA_INFO_SERIAL_NUMBER);
            const std::string name = get_camera_info(device, RS2_CAMERA_INFO_NAME);
            if (device_serial.empty()) {
                continue;
            }
            if (!preferred_serial.empty() && device_serial == preferred_serial) {
                serial = device_serial;
                device_name = name;
                return true;
            }
            if (name.find("RealSense") != std::string::npos) {
                realsense_devices.emplace_back(device_serial, name);
            }
            if (is_d455_device(device)) {
                d455_devices.emplace_back(device_serial, name);
            }
        }

        if (!preferred_serial.empty()) {
            std::cerr << "Preferred D455 serial from $" << kPreferredD455SerialEnv
                      << " not found: " << preferred_serial << std::endl;
            return false;
        }

        if (d455_devices.size() == 1) {
            serial = d455_devices.front().first;
            device_name = d455_devices.front().second;
            return true;
        }

        if (d455_devices.size() > 1) {
            std::ostringstream oss;
            oss << "Multiple D455 cameras detected, set $" << kPreferredD455SerialEnv << " to select one:";
            for (const auto& [device_serial, name] : d455_devices) {
                oss << " [" << name << " serial=" << device_serial << "]";
            }
            std::cerr << oss.str() << std::endl;
            return false;
        }

        if (realsense_devices.size() == 1) {
            serial = realsense_devices.front().first;
            device_name = realsense_devices.front().second;
            std::cout << "Warning: D455 not explicitly detected, falling back to the only available RealSense device"
                      << " serial=" << serial << std::endl;
            return true;
        }

        std::ostringstream oss;
        oss << "Unable to uniquely select a D455 camera.";
        for (const auto& [device_serial, name] : realsense_devices) {
            oss << " [" << name << " serial=" << device_serial << "]";
        }
        std::cerr << oss.str() << std::endl;
        return false;
    }
};

// Global control variables全局控制变量
std::atomic<bool> camera_connected{false};// 相机连接状态(原子布尔)
std::atomic<int> reconnect_attempts{0};     // 重连尝试次数(原子整数)
constexpr int MAX_RECONNECT = 500;          // 最大重连次数

// 计算从像素偏移量到角度的转换
std::pair<float, float> calculate_angles_from_offsets(
    float dx,// x方向偏移(像素)
    float dy,       // y方向偏移(像素)
    int image_width, // 图像宽度
    int image_height, // 图像高度
    float horizontal_fov, // 水平视场角
    float vertical_fov)   // 垂直视场角
{
    //  归一化像素偏移量(相对于图像中心)
    const float normalized_dx = dx / static_cast<float>(image_width / 2.0f);
    const float normalized_dy = dy / static_cast<float>(image_height / 2.0f);

    // Calculate angle offsets based on FOV基于视场角计算角度偏移
    const float yaw = normalized_dx * (horizontal_fov / 2.0f);// 偏航角
    const float pitch = -normalized_dy * (vertical_fov / 2.0f); //  俯仰角Invert Y-axis

    return {yaw, pitch};// 返回角度对
}
// 安全的角度计算函数(带参数验证)
std::pair<float, float> safe_calculate_angles(
    float dx, float dy,
    int image_width, int image_height,
    float horizontal_fov, float vertical_fov)
{
    // Parameter validation 参数验证
    if (image_width <= 0 || image_height <= 0)
        throw std::invalid_argument("Invalid image dimensions");
    
    if (horizontal_fov <= 0 || vertical_fov <= 0)
        throw std::invalid_argument("FOV values must be positive");

    // Calculate normalized offsets (add clamp to ensure safe range)// 限制偏移量在合理范围内
    const float clamped_dx = std::clamp(dx, -image_width/2.0f, image_width/2.0f);
    const float clamped_dy = std::clamp(dy, -image_height/2.0f, image_height/2.0f);
    
    // Reuse basic calculation logic调用基本计算函数
    auto [yaw, pitch] = calculate_angles_from_offsets(
        clamped_dx, clamped_dy, 
        image_width, image_height,
        horizontal_fov, vertical_fov);

    // Constrain angle ranges限制角度范围
    yaw = std::clamp(yaw, -horizontal_fov/2, horizontal_fov/2);
    pitch = std::clamp(pitch, -vertical_fov/2, vertical_fov/2);

    return {yaw, pitch}; //返回安全的角度值
}
// 发布检测结果到DDS，检测对象列表
void publish_detection_results(
    const vector<Detection> objects,
    const rs2_intrinsics& intrinsics,
    dds::pub::DataWriter<DetectionModule::DetectionResults> & writer,
    std::uint64_t frame_id,
    double infer_ms) {
    DetectionModule::DetectionResults results;// 创建DDS结果对象
    // 遍历所有检测对象
    for (const auto& obj : objects) {
        DetectionModule::DetectionResult result;// 单个结果对象
     // 设置类别信息
        result.class_id(std::to_string(obj.class_id));
        result.class_name(SafeClassName(obj.class_id));
        result.score(obj.conf);// 置信度
       // 设置边界框坐标(左上和右下点)
        // Set detection box
        std::array<float, 4> box = {
            static_cast<float>(obj.bbox.x),
            static_cast<float>(obj.bbox.y),
            static_cast<float>(obj.bbox.x + obj.bbox.width),
            static_cast<float>(obj.bbox.y + obj.bbox.height)
        };
        result.box(box);
        // Calculate center point coordinates 计算中心点坐标
        const float u = obj.bbox.x + obj.bbox.width / 2.0f;
        const float v = obj.bbox.y + obj.bbox.height / 2.0f;
        const float optical_center_x = intrinsics.fx > 1e-6f ? intrinsics.ppx : image_width / 2.0f;
        const float optical_center_y = intrinsics.fy > 1e-6f ? intrinsics.ppy : image_height / 2.0f;
        float offset_x = u - optical_center_x;// x方向偏移
        float offset_y = v - optical_center_y; // y方向偏移

        // Set 3D coordinates设置3D坐标
        std::array<float, 3> xyz = {obj.XYZ.x, obj.XYZ.y, obj.XYZ.z};
        result.xyz(xyz);
        // Set offsets 设置像素偏移量
        std::array<float, 2> offset = {offset_x, offset_y};
        result.offset(offset);
            // 计算并设置FOV偏移角度，优先使用相机自身内参
        auto yaw_pitch = calculate_angles_from_intrinsics_or_fov(
            u, v, image_width, image_height, intrinsics);
        std::array<float, 2> offset_fov = {yaw_pitch.first, yaw_pitch.second};
        result.offset_fov(offset_fov);
         // 添加到结果列表
        results.results().push_back(result);
    }

    writer.write(results);// 发布结果
    const int log_every_n = read_env_int("ROBOCUP_DETECT_LOG_EVERY_N", 1);
    if (log_every_n > 0 && frame_id % static_cast<std::uint64_t>(log_every_n) == 0) {
        std::cout << std::fixed << std::setprecision(3)
                  << "Detection frame=" << frame_id
                  << " objects=" << objects.size()
                  << " infer_ms=" << infer_ms << std::endl;
        for (const auto& obj : objects) {
            std::cout << std::fixed << std::setprecision(3)
                      << "  " << SafeClassName(obj.class_id)
                      << " score=" << obj.conf
                      << " box=[" << obj.bbox.x << "," << obj.bbox.y << ","
                      << obj.bbox.x + obj.bbox.width << ","
                      << obj.bbox.y + obj.bbox.height << "]"
                      << " xyz=[" << obj.XYZ.x << "," << obj.XYZ.y << "," << obj.XYZ.z << "]"
                      << std::endl;
        }
    }
}

float detection_range_m(const Detection& obj) {
    if (!std::isfinite(obj.XYZ.x) || !std::isfinite(obj.XYZ.z)) {
        return 0.0f;
    }
    return std::sqrt(obj.XYZ.x * obj.XYZ.x + obj.XYZ.z * obj.XYZ.z);
}

std::vector<Detection> filter_detections_for_runtime(const std::vector<Detection>& objects) {
    std::vector<Detection> filtered;
    filtered.reserve(objects.size());
    for (const auto& obj : objects) {
        if (g1DetectDisplayAccepts(SafeClassName(obj.class_id), obj.conf,
                                   detection_range_m(obj))) {
            filtered.push_back(obj);
        }
    }
    return filtered;
}

double current_head_yaw_deg(
    const std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>>& servo_state) {
    if (servo_state == nullptr || servo_state->msg_.states().size() <= kHeadYawIndex) {
        return 0.0;
    }
    return static_cast<double>(servo_state->msg_.states()[kHeadYawIndex].q());
}

double current_head_pitch_deg(
    const std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>>& servo_state) {
    if (servo_state == nullptr || servo_state->msg_.states().size() <= kHeadPitchIndex) {
        return 0.0;
    }
    return static_cast<double>(servo_state->msg_.states()[kHeadPitchIndex].q());
}

double current_waist_yaw_rad(
    const std::shared_ptr<unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::LowState_>>& low_state) {
    if (low_state == nullptr || low_state->msg_.motor_state().size() <= kWaistYawIndex) {
        return 0.0;
    }
    return static_cast<double>(low_state->msg_.motor_state()[kWaistYawIndex].q());
}

// Image processing thread  图像处理线程函数  // 相机控制器引用// YOLO模型引用// DDS写入器// 是否显示图像的标志
void processing_loop(
    CameraController& camera,
    YOLO& model,
    dds::pub::DataWriter<DetectionModule::DetectionResults>& detection_writer,
    dds::pub::DataWriter<LocationModule::LocationResult>& location_writer,
    RgbdLocalizer& rgbd_localizer,
    const std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>>& servo_state,
    const std::shared_ptr<unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::LowState_>>& low_state,
    string show_image_flag) {
    int specific_class_id = 0; //// 特定类别ID(0表示特殊处理)
    float conf_flag = read_env_float("ROBOCUP_DETECT_MIN_SCORE", 0.4f);      // 置信度阈值
    std::uint64_t frame_id = 0;
    std::cout << "Detection postprocess min_score=" << conf_flag
              << " log_every_n=" << read_env_int("ROBOCUP_DETECT_LOG_EVERY_N", 1)
              << std::endl;
    int consecutive_frame_misses = 0;
    constexpr int kMaxConsecutiveFrameMissesBeforeReconnect = 5;
    // 创建对齐对象(将深度图对齐到彩色图)
    rs2::align align(RS2_STREAM_COLOR);
    // DetectionResults results;
    while(true) {
         // 检查相机连接状态
        if(!camera_connected) {
            std::this_thread::sleep_for(100ms);
             // 发布无效结果(表示无连接)
            DetectionModule::DetectionResults results;
            DetectionModule::DetectionResult result;
            result.class_id(std::to_string(-1));// 无效ID
            result.class_name("");                // 空名称
            result.score(-1.0f);                  // 无效分数
    
        // 设置无效的边界框
            std::array<float, 4> box = {
                static_cast<float>(-1),
                static_cast<float>(-1),
                static_cast<float>(-1),
                static_cast<float>(-1)
            };
            result.box(box);
            // 设置无效的3D坐标
            std::array<float, 3> xyz = {-1, -1,-1};
            result.xyz(xyz);
            // 设置无效的偏移量
            std::array<float, 2> offset = {-1,-1};
            result.offset(offset);
               // 设置无效的FOV偏移
            std::array<float, 2> offset_fov = {-1,-1};
            result.offset_fov(offset_fov);

            results.results().push_back(result);
            detection_writer.write(results);// 发布无效结果
            continue;
        }
// 获取帧数据
        auto frameset = camera.poll_frames();
        if(!frameset) {
            ++consecutive_frame_misses;
            std::cerr << "No RealSense frames received, miss "
                      << consecutive_frame_misses << "/"
                      << kMaxConsecutiveFrameMissesBeforeReconnect
                      << std::endl;
            if (consecutive_frame_misses >= kMaxConsecutiveFrameMissesBeforeReconnect) {
                camera_connected = false;// 连续多次拿不到帧才标记相机断开
                consecutive_frame_misses = 0;
            }
            continue;
        }
        consecutive_frame_misses = 0;

        
        // 对齐深度和彩色帧
        auto aligned = align.process(frameset);
        auto color_frame = aligned.get_color_frame();// 获取彩色帧
        auto depth_frame = aligned.get_depth_frame();// 获取深度帧

   // 转换为OpenCV格式；使用实际帧尺寸，避免分辨率回退或 align 后尺寸变化导致 Mat 解释错误。
        cv::Mat image(
            cv::Size(color_frame.as<rs2::video_frame>().get_width(),
                     color_frame.as<rs2::video_frame>().get_height()),
            CV_8UC3,
            (void*)color_frame.get_data(),
            cv::Mat::AUTO_STEP);
        cv::Mat depth_image(
            cv::Size(depth_frame.as<rs2::video_frame>().get_width(),
                     depth_frame.as<rs2::video_frame>().get_height()),
            CV_16UC1,
            (void*)depth_frame.get_data(),
            cv::Mat::AUTO_STEP);
        if (image.empty()) break;// 检查空图像
        vector<Detection> objects;// 检测结果容器
        model.preprocess(image);  // 预处理图像
    // 执行推理并计时
        auto start = std::chrono::system_clock::now();
        model.infer();
        auto end = std::chrono::system_clock::now();
  // 后处理(过滤低置信度结果)
        model.postprocess(objects,depth_image,camera.intrinsics,conf_flag,specific_class_id);
        std::vector<Detection> runtime_objects = filter_detections_for_runtime(objects);
        // 显示框和发布给策略/定位的阈值保持一致：
        // Ball 使用 ROBOCUP_G1_BALL_MIN_SCORE，L/T/X 使用 marker locator 的置信度阈值。
        model.draw(image, runtime_objects);
       // 计算推理时间
        auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.;
            // 发布检测结果
        publish_detection_results(runtime_objects, camera.intrinsics, detection_writer, ++frame_id, tc);

        rgbd_localizer.update(
            image,
            depth_image,
            camera.intrinsics,
            camera.depth_scale_m,
            current_head_yaw_deg(servo_state),
            current_head_pitch_deg(servo_state),
            current_waist_yaw_rad(low_state));
        if (rgbd_localizer.has_pose()) {
            location_writer.write(rgbd_localizer.pose());
        }
        // 根据标志决定是否显示图像
        if (show_image_flag=="1")
        {
            imshow("prediction", image);
            if(cv::waitKey(1) == 27) break;
        }
    }
}
// 相机重连监控线程
void reconnect_monitor(CameraController& camera) {
    while(true) {
        // 如果已连接或达到最大重连次数，则等待
        if(camera_connected || reconnect_attempts >= MAX_RECONNECT) {
            std::this_thread::sleep_for(1s);
            continue;
        }
// 打印重连信息
        std::cout << "Attempting to reconnect (" << ++reconnect_attempts << "/" 
                  << MAX_RECONNECT << ")...\n";
// 先关闭相机
        camera.shutdown();
        // 尝试重新初始化
        if(camera.initialize()) {
            reconnect_attempts = 0;// 重置计数器
            camera_connected = true; // 标记为已连接
            std::cout << "Camera reconnected successfully! Serial number: " 
                      << camera.get_serial() << std::endl;
        } else {
            std::this_thread::sleep_for(2s);// 失败后等待
        }
    }
}
// 主函数
int main(int argc, char** argv)
{
     // 检查参数数量
    assert(argc >=2);
    // 获取模型文件路径
    const string engine_file_path{ argv[1] };
    string show_image_flag = "0";
    std::string network_interface = get_env_or_empty(kNetworkInterfaceEnv);
    if (network_interface.empty()) {
        network_interface = "eth0";
    }
    // 处理可选参数(是否显示图像)
    if (argc >= 3) {
        show_image_flag= argv[2] ;
    }
    if (argc >= 4) {
        network_interface = argv[3];
    }
    // 检查是否是ONNX文件(需要转换)
    if (engine_file_path.find(".onnx") == std::string::npos) // 加载TensorRT引擎
    {
        YOLO model(engine_file_path, logger);
        unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

        auto servo_state =
            std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>>("rt/g1_comp_servo/state");
        servo_state->msg_.states().resize(2);
        auto low_state =
            std::make_shared<unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::LowState_>>("rt/lowstate");

       // 初始化DDS
        dds::domain::DomainParticipant participant{0};// 创建域参与者
        // Register type and create Topic创建主题和质量策略
        dds::topic::qos::TopicQos topic_qos = dds::topic::qos::TopicQos();
        dds::topic::Topic<DetectionModule::DetectionResults> detection_topic(participant, "detectionresults", topic_qos);
        std::string rgbd_location_topic_name = get_env_or_empty(kRgbdLocationTopicEnv);
        if (rgbd_location_topic_name.empty()) {
            rgbd_location_topic_name = kDefaultRgbdLocationTopic;
        }
        std::cout << "RGB-D localizer publishes topic: " << rgbd_location_topic_name << std::endl;
        dds::topic::Topic<LocationModule::LocationResult> location_topic(participant, rgbd_location_topic_name, topic_qos);
        // 创建发布者和数据写入器
        dds::pub::Publisher publisher{participant};
        dds::pub::DataWriter<DetectionModule::DetectionResults> detection_writer{publisher, detection_topic};
        dds::pub::DataWriter<LocationModule::LocationResult> location_writer{publisher, location_topic};
        // 初始化相机
        CameraController camera;
        if(!camera.initialize()) {
            std::cerr << "Initialization failed, exiting program" << std::endl;
            return 1;
        }
        camera_connected = true;// 标记为已连接
        RgbdLocalizer rgbd_localizer;

        // Start processing threads启动处理线程
        std::thread proc_thread(
            std::bind(
                processing_loop,
                std::ref(camera),
                std::ref(model),
                std::ref(detection_writer),
                std::ref(location_writer),
                std::ref(rgbd_localizer),
                servo_state,
                low_state,
                std::ref(show_image_flag)));
         // 启动重连监控线程
        std::thread reconnect_thread(reconnect_monitor, std::ref(camera));

        // Wait for threads to finish等待线程结束
        proc_thread.join();
        reconnect_thread.join();
    }else// 如果是ONNX文件，转换为TensorRT引擎
    {
        std::cout << "Converting onnx to engine..." <<std::endl;
        YOLO model(engine_file_path, logger);
    }
    return 0;
}
