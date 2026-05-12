#ifndef UT_DYNAMIXEL_H
#define UT_DYNAMIXEL_H

#include "param.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <string>
#include <unordered_map>

#define MOTOR_NUM 2


namespace dxl
{
inline bool should_log_servo_warning(const std::string& key, double period_sec = 1.0)
{
    using clock = std::chrono::steady_clock;
    static std::mutex mutex;
    static std::unordered_map<std::string, clock::time_point> last_log_time;

    const auto now = clock::now();
    std::lock_guard<std::mutex> lock(mutex);
    auto it = last_log_time.find(key);
    if (it == last_log_time.end() ||
        std::chrono::duration<double>(now - it->second).count() >= period_sec)
    {
        last_log_time[key] = now;
        return true;
    }
    return false;
}

template <int num>
class Motors
{
    // Device setting
    constexpr static uint8_t  PROTOCOL_VERSION          = 2.0;

    // Control table address
    constexpr static uint16_t ADDR_OPERATING_MODE       = 11;
    constexpr static uint16_t ADDR_TORQUE_ENABLE        = 64; 
    constexpr static uint16_t POSITION_D_GAIN           = 80; 
    constexpr static uint16_t POSITION_P_GAIN           = 84; 
    constexpr static uint16_t ADDR_GOAL_POSITION        = 116;
    constexpr static uint16_t ADDR_PRESENT_POSITION     = 132;
    constexpr static uint8_t  OPERATING_MODE_POSITION   = 3;

    // Data Byte Length
    constexpr static uint16_t LEN_PRO_GOAL_POSITION     = 4;
    constexpr static uint16_t LEN_PRO_PRESENT_POSITION  = 4;
public:
    Motors(uint32_t BAUDRATE_ = 4500000)
    : BAUDRATE(BAUDRATE_)
    {
        InitDevice();
    }

    /* Enable/Disable the motor */
    void enable(id_t id, mode_t mode = 1)
    {
        uint8_t dxl_error = 0;
        int comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, mode, &dxl_error);
        if(comm_result != COMM_SUCCESS || dxl_error != 0) {
            spdlog::error("Failed to set torque motor {} to {}! comm_result: {}, dxl_error: {}",
                          id, mode, comm_result, dxl_error);
        }
    }

    void set_position_control_mode(id_t id)
    {
        // Dynamixel operating mode can only be changed when torque is disabled.
        enable(id, 0);
        uint8_t dxl_error = 0;
        int comm_result = packetHandler->write1ByteTxRx(
            portHandler, id, ADDR_OPERATING_MODE, OPERATING_MODE_POSITION, &dxl_error);
        if(comm_result != COMM_SUCCESS || dxl_error != 0) {
            spdlog::error("Failed to set motor {} operating mode to position! comm_result: {}, dxl_error: {}",
                          id, comm_result, dxl_error);
        } else {
            spdlog::info("Motor {} operating mode set to position.", id);
        }
    }

    bool read_torque_enabled(id_t id)
    {
        uint8_t dxl_error = 0;
        uint8_t torque_enabled = 0;
        int comm_result = packetHandler->read1ByteTxRx(
            portHandler, id, ADDR_TORQUE_ENABLE, &torque_enabled, &dxl_error);
        if(comm_result != COMM_SUCCESS || dxl_error != 0) {
            spdlog::error("Failed to read torque state motor {}! comm_result: {}, dxl_error: {}",
                          id, comm_result, dxl_error);
            return false;
        }
        return torque_enabled != 0;
    }

    // 单电机设定目标位置
    void set_position(id_t id, int goal_position)
    {
        uint8_t dxl_error = 0;
        int comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, goal_position, &dxl_error);
        if(comm_result != COMM_SUCCESS || dxl_error != 0) {
            if (should_log_servo_warning("set_position_" + std::to_string(id))) {
                spdlog::warn("Failed to set motor {} position {}! comm_result: {}, dxl_error: {}",
                             id, goal_position, comm_result, dxl_error);
            }
        }
    }

    // 单电机获取当前位置
    void get_position(id_t id, uint32_t* present_position)
    {
        uint8_t dxl_error = 0;
        if(packetHandler->read4ByteTxRx(portHandler, id, ADDR_PRESENT_POSITION, present_position, &dxl_error) != COMM_SUCCESS) {
            if (should_log_servo_warning("get_position_" + std::to_string(id))) {
                spdlog::warn("Failed to get position!");
            }
        }
    }

    // 多电机同步设定目标位置
    void sync_set_position(const std::array<int32_t, num>& goal_position)
    {
        uint8_t param_goal_position[4];
        for (id_t id = 0; id < goal_position.size(); id++)
        {
            param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_position[id]));
            param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_position[id]));
            param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_position[id]));
            param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_position[id]));

            if(!groupSyncWrite->addParam(id, (uint8_t*)&param_goal_position)) { 
                if (should_log_servo_warning("sync_write_addparam_" + std::to_string(id))) {
                    spdlog::warn("[ID:{}] GroupSyncWrite addparam failed!", id);
                }
            }
        }
        if(groupSyncWrite->txPacket() != COMM_SUCCESS) { 
            if (should_log_servo_warning("sync_write_tx")) {
                spdlog::warn("GroupSyncWrite txPacket failed!");
            }
        }
        groupSyncWrite->clearParam();
    }

    // 多电机同步获取当前位置
    void sync_get_position()
    {
        if(groupSyncRead->txRxPacket() != COMM_SUCCESS) {
            if (should_log_servo_warning("sync_read_txrx")) {
                spdlog::warn("GroupSyncRead txRxPacket failed!");
            }
            return;
        }

        for (id_t id = 0; id < present_position.size(); id++)
        {
            if(!groupSyncRead->isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)) {
                if (should_log_servo_warning("sync_read_available_" + std::to_string(id))) {
                    spdlog::warn("[ID:{}] Failed to get position!", id);
                }
                continue;
            }
            present_position[id] = groupSyncRead->getData(id, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        }
    }

    void set_position_p_gain(id_t id, uint16_t kp)
    {
        while (true) // 暂时采用死循环设置
        {
            if(_set_uint16_t(id, POSITION_P_GAIN, kp) == COMM_SUCCESS) break;
            usleep(50000);
        }
    }

    void set_position_d_gain(id_t id, uint16_t kd)
    {
        while (true)
        {
            if(_set_uint16_t(id, POSITION_D_GAIN, kd) == COMM_SUCCESS) break;
            usleep(50000);
        }
    }

    std::array<int32_t, num> present_position{};
private:
    int _set_uint16_t(id_t id, uint16_t address, uint16_t data)
    {
        uint8_t dxl_error = 0;
        const int comm_result = packetHandler->write2ByteTxRx(portHandler, id, address, data, &dxl_error);
        if (comm_result != COMM_SUCCESS || dxl_error != 0) {
            if (should_log_servo_warning("write_uint16_" + std::to_string(id) + "_" + std::to_string(address))) {
                spdlog::warn("Failed to write uint16 motor {} addr {} data {}! comm_result: {}, dxl_error: {}",
                             id, address, data, comm_result, dxl_error);
            }
        }
        return comm_result == COMM_SUCCESS && dxl_error == 0 ? COMM_SUCCESS : COMM_RX_FAIL;
    }


    /* Initialize the device */
    void InitDevice()
    {
        portHandler = dynamixel::PortHandler::getPortHandler(param::device.c_str());
        packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
        groupSyncWrite = std::make_shared<dynamixel::GroupSyncWrite>(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
        groupSyncRead = std::make_shared<dynamixel::GroupSyncRead>(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

        /* --- Open Device --- */
        if(!portHandler->openPort()) {
            spdlog::error("Failed to open the port {}!", param::device);
            exit(1);
        } else {
            spdlog::debug("Succeed to open the port {}!", param::device);
        }
        if(!portHandler->setBaudRate(BAUDRATE)) {
            spdlog::error("Failed to set the baudrate {}!", BAUDRATE);
            exit(1);
        } else {
            spdlog::debug("Succeed to set the baudrate {}!", BAUDRATE);
        }

        /* --- Sync Read/Write Add motor --- */
        for (id_t id = 0; id < num; id++)
        {
            if(!groupSyncRead->addParam(id)) { 
                spdlog::error("GroupSyncRead Failed to add motor {}!", id); }
        }
    }

    dynamixel::PortHandler * portHandler;
    dynamixel::PacketHandler * packetHandler;
    std::shared_ptr<dynamixel::GroupSyncWrite> groupSyncWrite;
    std::shared_ptr<dynamixel::GroupSyncRead> groupSyncRead;

    // config
    uint32_t BAUDRATE;
};

class g1Head : public Motors<2>
{
public:
    g1Head() : Motors<2>(57600) {};

    void Send()
    {
        /* --- Send --- */
        std::array<int32_t, MOTOR_NUM> goal_position;
        goal_position[0] = cmd.q[0] * (M_PI/180) * direction[0] / (2 * M_PI) * 4096;
        goal_position[1] = cmd.q[1] * (M_PI/180) * direction[1] / (2 * M_PI) * 4096;
        this->sync_set_position(goal_position);
    }

    void Recv()
    {
        this->sync_get_position();
        
        /* --- Recv --- */
        state.q[0] = present_position[0] * ((2 * M_PI) / 4096) * (180/M_PI);
        state.q[1] = present_position[1] * ((2 * M_PI) / 4096) * (180/M_PI);
    }

    struct {
        Eigen::Matrix<float, MOTOR_NUM, 1> q{};
    } cmd;
    struct {
        Eigen::Matrix<float, MOTOR_NUM, 1> q{};
    } state;
private:
    std::array<float, MOTOR_NUM> direction = {1, 1};
};

} // namespace dxl

#endif // UT_DYNAMIXEL_H
