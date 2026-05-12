#include <algorithm>
#include <cstdlib>
#include <string>

#include "dxl.h"
#include "dds/Publisher.h"
#include "dds/Subscription.h"
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include "utilities.h"
#include "yaml_parser.h"


class IOUSB
{
public:
    static constexpr float kControlPeriodSec = 0.015f;

    IOUSB()
        : joints(1000000)
    {
        motorstate = std::make_unique<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>>(
            "rt/g1_comp_servo/state");
        motorstate->msg_.states().resize(2);
        motorcmd = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>>(
            "rt/g1_comp_servo/cmd");
        motorcmd->msg_.cmds().resize(2);


        for (int i(0); i<2; i++)
        {
            joints.set_position_control_mode(i);
            joints.set_position_p_gain(i, 500);
            joints.set_position_d_gain(i, 300);
        }
        spdlog::info("Set gain success.");

        const char* cfg_env = std::getenv("ROBOCUP_SERVO_CONFIG");
        std::string env_cfg_path = (cfg_env != nullptr && cfg_env[0] != '\0')
            ? std::string(cfg_env)
            : std::string("../config/config.yaml");
        spdlog::info("Servo config: {}", env_cfg_path);
        yamlParam_.setup(env_cfg_path.c_str());

        float has_calibrate = yamlParam_.ReadFloatFromYaml("has_calibrate");
        if(!has_calibrate) 
        {
            std::cout<<std::endl;
            std::cout<<" please run test_calibration first! "<<std::endl;
            exit(-1);
        }

        servo0_calibration = yamlParam_.ReadFloatFromYaml("servo0_calibration");
        servo1_calibration = yamlParam_.ReadFloatFromYaml("servo1_calibration");

        joint0_limitation.setZero(2);
        joint1_limitation.setZero(2);
        
        joint0_limitation = yamlParam_.ReadVectorFromYaml("joint0",2);
        joint1_limitation = yamlParam_.ReadVectorFromYaml("joint1",2);

        servo0_limit_enoder = yamlParam_.ReadFloatFromYaml("servo0_calibration");
        servo1_limit_enoder = yamlParam_.ReadFloatFromYaml("servo1_calibration");

        direction.setZero(2);
        direction = yamlParam_.ReadVectorFromYaml("direction",2);

        servo_angle.setZero(2);
        target_angle_command.setZero(2);
        startup_hold_position_deg_.setZero(2);
        max_speed_deg_per_sec_.setZero(2);

        startup_hold_enabled_ = yamlParam_.ReadFloatFromYaml("startup_hold_enabled") > 0.5f;
        startup_hold_position_deg_ = yamlParam_.ReadVectorFromYaml("startup_hold_position_deg",2);
        max_speed_deg_per_sec_ = yamlParam_.ReadVectorFromYaml("max_speed_deg_per_sec",2);

        target_angle_command = startup_hold_position_deg_;
        spdlog::info(
            "Startup hold: enabled={}, yaw={} deg, pitch={} deg, max_speed=[{}, {}] deg/s",
            startup_hold_enabled_,
            startup_hold_position_deg_(0),
            startup_hold_position_deg_(1),
            max_speed_deg_per_sec_(0),
            max_speed_deg_per_sec_(1));
    }

    void Start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
           0.015 * 1000, std::bind(&IOUSB::run, this));
    }
    dxl::Motors<2> joints;

private:
    void run()
    {
        const bool has_external_command =
            motorcmd->msg_.cmds()[0].mode() == 1 || motorcmd->msg_.cmds()[1].mode() == 1;
        if (has_external_command) {
            external_command_seen_ = true;
        }

        const bool should_hold_position = startup_hold_enabled_ || external_command_seen_;
        check_motor_enable(should_hold_position || has_external_command);
        
        for(int i(0); i<2; i++)
        {   
            float desired_angle_deg = target_angle_command(i);
            if (has_external_command) {
                desired_angle_deg = motorcmd->msg_.cmds()[i].q();
            } else if (!external_command_seen_ && startup_hold_enabled_) {
                desired_angle_deg = startup_hold_position_deg_(i);
            }

            const float max_delta = std::max(0.0f, max_speed_deg_per_sec_(i)) * kControlPeriodSec;
            const float delta = std::clamp(desired_angle_deg - target_angle_command(i), -max_delta, max_delta);
            target_angle_command(i) += delta;

            float servo_encoder_positon;
            if(i == 0)
                servo_encoder_positon = utilities::angle2encoder(target_angle_command(i),servo0_limit_enoder,joint0_limitation,direction(0));
            if(i == 1)
                servo_encoder_positon = utilities::angle2encoder(target_angle_command(i),servo1_limit_enoder,joint1_limitation,direction(1));
            joints.set_position(i, servo_encoder_positon);
        }
        
        joints.sync_get_position();
        servo_angle(0) = utilities::encoder2angle(joints.present_position[0],servo0_limit_enoder,joint0_limitation,direction(0));
        servo_angle(1) = utilities::encoder2angle(joints.present_position[1],servo1_limit_enoder,joint1_limitation,direction(1));
        
        
        for(int i(0); i<2; i++)
        {
            motorstate->msg_.states()[i].q(servo_angle(i));
        }
        motorstate->unlockAndPublish();

    }

    void check_motor_enable(bool should_enable)
    {
        if(should_enable)
        {
            if(!joint_enable)
            {
                for(int i(0); i<MOTOR_NUM; i++)
                {
                    joints.enable(i);
                    spdlog::info("Motor {} torque state after enable: {}", i, joints.read_torque_enabled(i));
                }
                joint_enable = true;
                spdlog::info("Enable arm.");
            }
        }
        else
        {
            if(joint_enable)
            {
                for(int i(0); i<MOTOR_NUM; i++)
                {
                    joints.enable(i, 0);
                }
                joint_enable = false;
                spdlog::info("Release arm.");
            }
        }
    }

    unitree::common::RecurrentThreadPtr thread_;
    bool joint_enable = false;
    bool startup_hold_enabled_ = true;
    bool external_command_seen_ = false;

    /* DDS */
    std::unique_ptr<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>> motorstate; // motor state
    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>> motorcmd; // motor cmd
    YamlParser yamlParam_;
    float servo0_calibration;
    float servo1_calibration;
    Eigen::VectorXf joint0_limitation;
    Eigen::VectorXf joint1_limitation;
    float servo0_limit_enoder;
    float servo1_limit_enoder;
    Eigen::VectorXf direction;
    Eigen::VectorXf servo_angle;
    Eigen::VectorXf startup_hold_position_deg_;
    Eigen::VectorXf max_speed_deg_per_sec_;

    Eigen::VectorXf target_angle_command;
    Eigen::VectorXf target_encoder_command;
};

int main(int argc, char** argv)
{
    auto vm = param::helper(argc, argv);
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    auto iousb = IOUSB();
    iousb.Start();
    while(true)
    {
        sleep(1);
    }
    return 0;
}
