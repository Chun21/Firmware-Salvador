#include "game_controller.h"

#include <boost/asio.hpp>
#include <chrono>
#include <algorithm>
#include <cstdlib>
#include <limits>
#include <loguru.hpp>
#include <thread>

#include "RoboCupGameControlData.h"
#include "gc_pub_sub.h"
#include "named_thread.h"
#include "robot_time.h"
#include "stl_ext.h"

GameController::GameController(uint8_t team_nr, uint8_t player_idx)
    : team_nr(team_nr), player_idx(player_idx) {
    GCState initial_state;
    initial_state.player_idx = player_idx;
    initial_state.my_team.team_nr = team_nr;
    initial_state.opp_team.team_nr = 0;
    initial_state.my_team.players.resize(std::max<int>(5, player_idx + 1));
    initial_state.opp_team.players.resize(std::max<int>(5, player_idx + 1));
    initial_state.game_phase = GCState::GamePhase::FIRST_HALF;
    initial_state.state = GameState::Initial;
    initial_state.secondary_state = SecondaryState::Normal;
    initial_state.kicking_team = KickingTeam::Unknown;
    initial_state.remaining_message_budget = std::numeric_limits<uint16_t>::max();
    gc_state.publish(initial_state);
    state = initial_state;

    controller_thread = named_thread("game_controller", &GameController::run, this);
    heartbeat_thread = named_thread("gc_heartbeat", &GameController::send_heartbeats, this);
}

GameController::~GameController() {
    running = false;

    if (controller_thread.joinable()) {
        controller_thread.join();
    }

    if (heartbeat_thread.joinable()) {
        heartbeat_thread.join();
    }
}

void GameController::handle_socket_error(const char* op, const boost::system::error_code& ec) {
    if (ec) {
        LOG_F(FATAL, "%s socket failed: %s.", op, ec.message().c_str());
    }
}

void GameController::run() {
    boost::asio::io_context io_context;
    boost::system::error_code ec;
    boost::asio::ip::udp::socket socket(io_context);

    handle_socket_error("open", socket.open(boost::asio::ip::udp::v4(), ec));
    handle_socket_error("set_option",
                        socket.set_option(boost::asio::socket_base::reuse_address(true), ec));
    handle_socket_error("bind",
                        socket.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(),
                                                                   GAMECONTROLLER_DATA_PORT),
                                    ec));

    // Set socket to non-blocking mode to allow checking shutdown flag
    socket.non_blocking(true, ec);
    if (ec) {
        LOG_F(ERROR, "Failed to set socket non-blocking: %s", ec.message().c_str());
    }

    RoboCupGameControlData gcData{};
    while (running) {
        boost::asio::ip::udp::endpoint remoteEndpoint;
        size_t len = socket.receive_from(boost::asio::buffer(&gcData, sizeof(gcData)),
                                         remoteEndpoint, 0, ec);

        if (ec) {
            if (ec == boost::asio::error::would_block) {
                // No data available, sleep briefly and continue
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            LOG_F(ERROR, "receive_from failed: %s", ec.message().c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            continue;
        }

        if (std::strncmp(gcData.header, GAMECONTROLLER_STRUCT_HEADER,
                         strlen(GAMECONTROLLER_STRUCT_HEADER)) != 0) {
            std::string r_header(gcData.header, sizeof(gcData.header));
            LOG_F(ERROR, "Received GameController packet with wrong header: got %s, expected %s",
                  r_header.c_str(), GAMECONTROLLER_STRUCT_HEADER);
            continue;
        }

        if (gcData.version != GAMECONTROLLER_STRUCT_VERSION) {
            LOG_F(ERROR, "Received GameController packet with wrong version: got %d, expected %d",
                  gcData.version, GAMECONTROLLER_STRUCT_VERSION);
            continue;
        }

        if (len < sizeof(RoboCupGameControlData)) {
            LOG_F(ERROR,
                  "Received GameController packet with wrong length: got %zu, expected at least "
                  "%zu",
                  len, sizeof(RoboCupGameControlData));
            continue;
        }

        if (gcData.teams[0].teamNumber != team_nr && gcData.teams[1].teamNumber != team_nr)
            continue;

        if (!isNewPackage(gcData))
            continue;

        GCState new_state;
        new_state.player_idx = player_idx;

        int my_team_idx = gcData.teams[0].teamNumber == team_nr ? 0 : 1;
        // Not our game.
        if (gcData.teams[my_team_idx].teamNumber != team_nr)
            continue;
        new_state.my_team.team_nr = team_nr;
        // TODO: Implement substitudes better.
        const int player_count = std::clamp<int>(
                std::max<int>(gcData.playersPerTeam, player_idx + 1), 1, MAX_NUM_PLAYERS);
        for (int i = 0; i < player_count; i++) {
            Player player;
            player.is_penalized = gcData.teams[my_team_idx].players[i].penalty != PENALTY_NONE;
            new_state.my_team.players.push_back(player);
        }
        new_state.opp_team.team_nr = gcData.teams[1 - my_team_idx].teamNumber;

        for (int i = 0; i < player_count; i++) {
            Player player;
            player.is_penalized = gcData.teams[1 - my_team_idx].players[i].penalty != PENALTY_NONE;
            new_state.opp_team.players.push_back(player);
        }

        SecondaryState secondary_state = map_set_play(gcData.setPlay);
        if (gcData.gamePhase == GAME_PHASE_PENALTY_SHOOT_OUT) {
            new_state.game_phase = GCState::GamePhase::PENALTY_SHOOT;
        } else if (gcData.firstHalf == 1) {
            new_state.game_phase = GCState::GamePhase::FIRST_HALF;
        } else {
            new_state.game_phase = GCState::GamePhase::SECOND_HALF;
        }
        new_state.state = static_cast<GameState>(gcData.state);
        new_state.secondary_state = secondary_state;
        if (gcData.kickingTeam == KICKING_TEAM_NONE)
            new_state.kicking_team = KickingTeam::Unknown;
        else
            new_state.kicking_team =
                    gcData.kickingTeam == team_nr ? KickingTeam::MyTeam : KickingTeam::OppTeam;
        new_state.setPlay_kicking_team = state.setPlay_kicking_team;
        if (gcData.setPlay != SET_PLAY_NONE) {
            if (gcData.kickingTeam == KICKING_TEAM_NONE) {
                new_state.setPlay_kicking_team = KickingTeam::Unknown;
            } else {
                new_state.setPlay_kicking_team = gcData.kickingTeam == team_nr
                                                         ? KickingTeam::MyTeam
                                                         : KickingTeam::OppTeam;
            }
            new_state.setPlay_state = new_state.state;
        } else {
            new_state.setPlay_kicking_team = KickingTeam::Unknown;
            new_state.setPlay_state = GameState::Initial;
        }
        new_state.secs_remaining = static_cast<uint16_t>(std::max<int16_t>(0, gcData.secsRemaining));
        new_state.secondary_time = static_cast<uint16_t>(std::max<int16_t>(0, gcData.secondaryTime));
        new_state.remaining_message_budget = gcData.teams[my_team_idx].messageBudget;

        {
            std::lock_guard<std::mutex> lock(heartbeat_mutex);
            boost::asio::ip::udp::endpoint endpoint = remoteEndpoint;
            endpoint.port(GAMECONTROLLER_RETURN_PORT);
            heartbeat_endpoint = endpoint;
        }
        LOG_S(ERROR) << "GameState received: " << (int)new_state.state
                     << " Secondary: " << (int)new_state.secondary_state
                     << " SetPlay: " << (int)new_state.setPlay_state;
        gc_state.publish(new_state);

        if (state != new_state) {
            on_gc_state_change(state, new_state);
            state = new_state;
        }
    }
}

void GameController::send_heartbeats() {
    boost::asio::io_context io_context;
    boost::system::error_code ec;
    boost::asio::ip::udp::socket socket(io_context);

    handle_socket_error("open", socket.open(boost::asio::ip::udp::v4(), ec));

    RoboCupGameControlReturnData returnData;
    returnData.teamNum = team_nr;
    returnData.playerNum = player_idx + 1;

    while (running) {
        std::optional<boost::asio::ip::udp::endpoint> current_endpoint;

        {
            std::lock_guard<std::mutex> lock(heartbeat_mutex);
            current_endpoint = heartbeat_endpoint;
        }

        if (current_endpoint) {
            if (auto loc_position = loc_position_subscriber.latestIfExists()) {
                returnData.pose[0] = loc_position->position.x * 1000.0f;
                returnData.pose[1] = loc_position->position.y * 1000.0f;
                returnData.pose[2] = loc_position->position.a;
            }

            std::optional<RelBall> rel_ball = rel_ball_subscriber.latest();
            if (rel_ball) {
                returnData.ballAge =
                        std::max(0.0f, static_cast<float>(time_us() - rel_ball->last_seen_time) /
                                             1'000'000.0f);
                returnData.ball[0] = rel_ball->pos_rel.x * 1000.0f;
                returnData.ball[1] = rel_ball->pos_rel.y * 1000.0f;
            } else {
                returnData.ballAge = -1.0f;
                returnData.ball[0] = 0.0f;
                returnData.ball[1] = 0.0f;
            }

            const htwk::FallDownState fallen = fallen_subscriber.latest();
            returnData.fallen = fallen.type == htwk::FallDownStateType::READY ? 0 : 1;

            socket.send_to(boost::asio::buffer(&returnData, sizeof(returnData)), *current_endpoint,
                           0, ec);
            if (ec) {
                LOG_F(ERROR, "Failed to send heartbeat: %s", ec.message().c_str());
            }
        }

        // Sleep in smaller increments to allow checking shutdown flag more frequently
        for (int i = 0; i < 100 && running; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

bool GameController::isNewPackage(const RoboCupGameControlData& gcData) {
    if (has_last_packet && gcData.packetNumber == last_packet_number) {
        return false;
    }
    last_packet_number = gcData.packetNumber;
    has_last_packet = true;
    return true;
}

SecondaryState GameController::map_set_play(uint8_t set_play) {
    switch (set_play) {
        case SET_PLAY_NONE:
            return SecondaryState::Normal;
        case SET_PLAY_DIRECT_FREE_KICK:
            return SecondaryState::DirectFreeKick;
        case SET_PLAY_INDIRECT_FREE_KICK:
            return SecondaryState::IndirectFreeKick;
        case SET_PLAY_PENALTY_KICK:
            return SecondaryState::PenaltyKick;
        case SET_PLAY_THROW_IN:
            return SecondaryState::ThrowIn;
        case SET_PLAY_GOAL_KICK:
            return SecondaryState::GoalKick;
        case SET_PLAY_CORNER_KICK:
            return SecondaryState::CornerKick;
        default:
            return SecondaryState::Normal;
    }
}
