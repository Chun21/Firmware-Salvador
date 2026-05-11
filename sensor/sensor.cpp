#include "sensor.h"

#ifdef ROBOT_MODEL_G1

#include <cmath>
#include <functional>

#include "logging.h"
#include "odometer.h"
#include "robot_time.h"

namespace htwk {

namespace {

IMU_new toImu(const unitree_hg::msg::dds_::IMUState_& imu_msg) {
    IMU_new imu;
    imu.gyro[0] = imu_msg.gyroscope()[0];
    imu.gyro[1] = imu_msg.gyroscope()[1];
    imu.gyro[2] = imu_msg.gyroscope()[2];
    imu.rpy[0] = imu_msg.rpy()[0];
    imu.rpy[1] = imu_msg.rpy()[1];
    imu.rpy[2] = imu_msg.rpy()[2];
    imu.acc[0] = imu_msg.accelerometer()[0];
    imu.acc[1] = imu_msg.accelerometer()[1];
    imu.acc[2] = imu_msg.accelerometer()[2];
    return imu;
}

}  // namespace

Sensor::Sensor() {
    lowstate_subscriber =
            std::make_unique<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(
                    "rt/lowstate", std::bind_front(&Sensor::lowHandler, this));
    lowstate_subscriber->InitChannel();

    secondary_imu_subscriber =
            std::make_unique<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::IMUState_>>(
                    "rt/secondary_imu", std::bind_front(&Sensor::secondaryImuHandler, this));
    secondary_imu_subscriber->InitChannel();

    odom_subscriber = std::make_unique<
            unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(
            "rt/odommodestate", std::bind_front(&Sensor::odomHandler, this));
    odom_subscriber->InitChannel();

    fallen_channel.publish(FallDownState{});
    LOG_F(INFO, "G1 sensor initialized: lowstate, secondary IMU, odom");
}

Sensor::~Sensor() {
    if (lowstate_subscriber) {
        lowstate_subscriber->CloseChannel();
    }
    if (secondary_imu_subscriber) {
        secondary_imu_subscriber->CloseChannel();
    }
    if (odom_subscriber) {
        odom_subscriber->CloseChannel();
    }
}

void Sensor::lowHandler(const void* msg) {
    const auto* low_state_msg = static_cast<const unitree_hg::msg::dds_::LowState_*>(msg);
    updateFallDownState(toImu(low_state_msg->imu_state()));
}

void Sensor::secondaryImuHandler(const void* msg) {
    const auto* imu_msg = static_cast<const unitree_hg::msg::dds_::IMUState_*>(msg);
    updateFallDownState(toImu(*imu_msg));
}

void Sensor::odomHandler(const void* msg) {
    const auto* odom_msg = static_cast<const unitree_go::msg::dds_::SportModeState_*>(msg);
    const int64_t now = time_us();
    Odometer odometer_state;
    odometer_state.x = odom_msg->position()[0];
    odometer_state.y = odom_msg->position()[1];
    odometer_state.theta = odom_msg->imu_state().rpy()[2];
    odometer_state.timestamp_system_us = now;
    odometer_history.put(now, odometer_state);
    odometer_channel.publish(std::make_shared<Odometer>(odometer_state));
}

void Sensor::updateFallDownState(const IMU_new& imu) {
    FallDownState current = fallen_channel.create_subscriber().latest();
    if (current.type == FallDownStateType::READY && is_fallen(imu)) {
        current.type = FallDownStateType::FALLEN;
        current.side = get_fall_down_side(imu);
        LOG_F(ERROR, "G1 unsafe posture detected; entering FALLEN state");
        fallen_channel.publish(current);
    } else if (current.type == FallDownStateType::FALLEN && !is_fallen(imu)) {
        current.to_succeeded();
        LOG_F(INFO, "G1 posture back to READY");
        fallen_channel.publish(current);
    }
}

}  // namespace htwk

#else

#include <booster/idl/b1/LowCmd.h>
#include <booster/idl/b1/LowState.h>
#include <booster/idl/b1/MotorCmd.h>

#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/b1/b1_loco_client.hpp>
#include <booster/robot/channel/channel_subscriber.hpp>

#include "imu_joint_state.h"
#include "logging.h"
#include "odometer.h"
#include "robot_time.h"

namespace htwk {

void Sensor::odoHandler_from_imu(const IMUJointState& imu_joint_state) {
    float current_yaw = imu_joint_state.imu.rpy[2];
    float yaw_diff = current_yaw - last_odo_yaw;
    // Normalize angle difference to [-π, π]
    while (yaw_diff > M_PI) {
        yaw_diff -= 2.0 * M_PI;
    }
    while (yaw_diff < -M_PI) {
        yaw_diff += 2.0 * M_PI;
    }

    last_odo_yaw = current_yaw;

    int64_t time = time_us();
    float dx = 0.0f;
    float dy = 0.0f;

    odometer_history.put(time, Odometer{dx, dy, yaw_diff});
    // clear all that is older then 2 seconds
    if (odometer_history.size() > 2000) {
        odometer_history.clear_last_n(100);
    }
};

void Sensor::odoHandler(const void* msg) {
    Odometer odometer_state;
    const auto* odom_msg = static_cast<const booster_interface::msg::Odometer*>(msg);

    int64_t time = time_us();

    odometer_state.x = odom_msg->x();
    odometer_state.y = odom_msg->y();
    odometer_state.theta = odom_msg->theta();
    odometer_state.timestamp_system_us = time;

    odometer_history.put(time, Odometer{odometer_state.x, odometer_state.y, odometer_state.theta});
    // clear all that is older then 2 seconds
    if (odometer_history.size() > 2000) {
        odometer_history.clear_last_n(100);
    }
};

void Sensor::lowHandler(const void* msg) {
    const auto* low_state_msg = static_cast<const booster_interface::msg::LowState*>(msg);
    IMUJointState imu_joint_state;
    imu_joint_state.imu.gyro[0] = low_state_msg->imu_state().gyro()[0];
    imu_joint_state.imu.gyro[1] = low_state_msg->imu_state().gyro()[1];
    imu_joint_state.imu.gyro[2] = low_state_msg->imu_state().gyro()[2];
    imu_joint_state.imu.rpy[0] = low_state_msg->imu_state().rpy()[0];
    imu_joint_state.imu.rpy[1] = low_state_msg->imu_state().rpy()[1];
    imu_joint_state.imu.rpy[2] = low_state_msg->imu_state().rpy()[2];
    imu_joint_state.imu.acc[0] = low_state_msg->imu_state().acc()[0];
    imu_joint_state.imu.acc[1] = low_state_msg->imu_state().acc()[1];
    imu_joint_state.imu.acc[2] = low_state_msg->imu_state().acc()[2];

    // serial
    for (int i = 0; i < joint_cnt; ++i) {
        imu_joint_state.parallel[i].q = low_state_msg->motor_state_parallel()[i].q();
        imu_joint_state.parallel[i].dq = low_state_msg->motor_state_parallel()[i].dq();
        imu_joint_state.parallel[i].ddq = low_state_msg->motor_state_parallel()[i].ddq();
        imu_joint_state.parallel[i].tau_est = low_state_msg->motor_state_parallel()[i].tau_est();

        imu_joint_state.serial[i].q = low_state_msg->motor_state_serial()[i].q();
        imu_joint_state.serial[i].dq = low_state_msg->motor_state_serial()[i].dq();
        imu_joint_state.serial[i].ddq = low_state_msg->motor_state_serial()[i].ddq();
        imu_joint_state.serial[i].tau_est = low_state_msg->motor_state_serial()[i].tau_est();
    }
    /*auto gyro_arrows = rerun::Arrows3D::from_vectors({
               
    odoHandler_from_imu(imu_joint_state);                                              {1.0f, 0.0f, 0.0f},
                                                             {0.0f, 1.0f, 0.0f},
                                                             {0.0f, 0.0f, 1.0f},
                                                     })
                               .with_origins({{0, 0, 0}, {0, 0, 0}, {0, 0, 0}});*/
    // log raw IMU data
    LOG_F(OFFLINE, "imu_joint: %.*s", (int)sizeof(imu_joint_state), (const char*)&imu_joint_state);
    imu_joint_states_channel.publish(std::make_shared<IMUJointState>(imu_joint_state));

    odoHandler_from_imu(imu_joint_state);
    update_fall_down_state(imu_joint_state);
};

void Sensor::update_fall_down_state(IMUJointState imu_joint_state) {
    FallDownState current_fall_down_state = fallen_subscriber.latest();

    if (current_fall_down_state.type == htwk::FallDownStateType::READY &&
        is_fallen(imu_joint_state.imu)) {
        current_fall_down_state.type = htwk::FallDownStateType::FALLEN;
        current_fall_down_state.side = get_fall_down_side(imu_joint_state.imu);
        LOG_F(INFO, "Set to FALLEN, side: %s",
              current_fall_down_state.side == htwk::FallDownSide::FRONT ? "FRONT" : "BACK");
        fallen_channel.publish(current_fall_down_state);
    } else if (current_fall_down_state.type == htwk::FallDownStateType::GETTING_UP) {
        float total_torque = 0.0f;
        for (const auto& joint : imu_joint_state.parallel) {
            total_torque += std::abs(joint.tau_est);
        }
        int64_t get_up_time;
        float torque_limit;
        if (current_fall_down_state.side == htwk::FallDownSide::FRONT) {
            get_up_time = front_get_up_time;
            torque_limit = front_torque_limit;
        } else {
            get_up_time = back_get_up_time;
            torque_limit = back_torque_limit;
        }
        if (time_us() - last_getup_started > 1_s && total_torque < torque_limit) {
            LOG_F(INFO, "GetUp Failure detected due to torque");
            current_fall_down_state.to_failed(get_fall_down_side(imu_joint_state.imu));
            fallen_channel.publish(current_fall_down_state);
            return;
        }
        if (time_us() - last_getup_started > get_up_time) {
            LOG_F(INFO, "Check after GetUp");
            if (is_fallen(imu_joint_state.imu)) {
                LOG_F(INFO, "GetUp FAILED");
                current_fall_down_state.to_failed(get_fall_down_side(imu_joint_state.imu));
            } else {
                LOG_F(INFO, "GetUp SUCCEEDED");
                current_fall_down_state.to_succeeded();
            }
            fallen_channel.publish(current_fall_down_state);
        }
    }
}

void Sensor::update_fall_down_state_booster(const void* msg) {
    const auto* booster_fall_state = static_cast<const booster_interface::msg::FallDownState*>(msg);
    htwk::FallDownState current_fall_state = fallen_subscriber.latest();
    if (booster_fall_state->fall_down_state() == 2 &&
        current_fall_state.type != htwk::FallDownStateType::GETTING_UP) {
        last_getup_started = time_us();
        LOG_F(INFO, "Set to GETTING_UP");
        current_fall_state.type = htwk::FallDownStateType::GETTING_UP;
        fallen_channel.publish(current_fall_state);
        return;
    }
    if (booster_fall_state->fall_down_state() == 0 &&
        current_fall_state.type != htwk::FallDownStateType::READY) {
        LOG_F(INFO, "Set to READY");
        current_fall_state.type = htwk::FallDownStateType::READY;
        fallen_channel.publish(current_fall_state);
        return;
    }
}

Sensor::Sensor() {
    channel_subscriber_low =
            std::make_unique<booster::robot::ChannelSubscriber<booster_interface::msg::LowState>>(
                    booster::robot::b1::kTopicLowState, std::bind_front(&Sensor::lowHandler, this));
    channel_subscriber_low->InitChannel();
    /* enable if you take the booster odometer as source
    channel_subscriber_odo =
            std::make_unique<booster::robot::ChannelSubscriber<booster_interface::msg::Odometer>>(
                    booster::robot::b1::kTopicOdometerState,
                    std::bind_front(&Sensor::odoHandler, this));
    channel_subscriber_odo->InitChannel();
     */

    channel_subscriber_fall_down = std::make_unique<
            booster::robot::ChannelSubscriber<booster_interface::msg::FallDownState>>(
            booster::robot::b1::kTopicFallDown,
            std::bind_front(&Sensor::update_fall_down_state_booster, this));
    channel_subscriber_fall_down->InitChannel();

    LOG_F(INFO, "Sensor initialized");
}

Sensor::~Sensor() {
    if (channel_subscriber_low) {
        channel_subscriber_low->CloseChannel();
    }
    if (channel_subscriber_odo) {
        channel_subscriber_odo->CloseChannel();
    }
    if (channel_subscriber_fall_down) {
        channel_subscriber_fall_down->CloseChannel();
    }
}
}  // namespace htwk

#endif  // ROBOT_MODEL_G1
