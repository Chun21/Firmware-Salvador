#pragma once

#include <atomic>
#include <boost/asio.hpp>
#include <mutex>
#include <optional>
#include <thread>

#include "gc_state.h"
#include "localization_pub_sub.h"
#include "multi_target_tracker_pub_sub.h"
#include "sensor_pub_sub.h"

class RoboCupGameControlData;

class GameController : public GCInterface {
public:
    GameController(uint8_t team_nr, uint8_t player_idx);
    ~GameController() override;

private:
    bool isNewPackage(const RoboCupGameControlData& gcData);
    void run();
    void send_heartbeats();
    void handle_socket_error(const char* op, const boost::system::error_code& ec);
    static SecondaryState map_set_play(uint8_t set_play);

    uint8_t last_packet_number = 0;
    bool has_last_packet = false;
    uint8_t team_nr;
    uint8_t player_idx;
    GCState state;
    htwk::ChannelSubscriber<LocPosition> loc_position_subscriber =
            loc_position_channel.create_subscriber();
    htwk::ChannelSubscriber<std::optional<RelBall>> rel_ball_subscriber =
            rel_ball_channel.create_subscriber();
    htwk::ChannelSubscriber<htwk::FallDownState> fallen_subscriber =
            htwk::fallen_channel.create_subscriber();
    std::thread controller_thread;
    std::thread heartbeat_thread;
    std::optional<boost::asio::ip::udp::endpoint> heartbeat_endpoint;
    std::mutex heartbeat_mutex;
    std::atomic<bool> running{true};
};
