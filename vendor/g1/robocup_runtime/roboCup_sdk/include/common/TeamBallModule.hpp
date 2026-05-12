/***************************************************************
  TeamBallModule DDS type. Kept modular under roboCup_sdk/include/common.
****************************************************************/
#ifndef DDSCXX_TEAMBALLMODULE_HPP
#define DDSCXX_TEAMBALLMODULE_HPP

#include <cstdint>
#include <string>

namespace TeamBallModule
{
class TeamBall
{
private:
 std::string team_id_;
 std::string robot_id_;
 float ball_field_x_ = 0.0f;
 float ball_field_y_ = 0.0f;
 float confidence_ = 0.0f;
 float observer_field_x_ = 0.0f;
 float observer_field_y_ = 0.0f;
 float observer_field_theta_ = 0.0f;
 uint32_t seq_ = 0;

public:
  TeamBall() = default;

  explicit TeamBall(
    const std::string& team_id,
    const std::string& robot_id,
    float ball_field_x,
    float ball_field_y,
    float confidence,
    float observer_field_x,
    float observer_field_y,
    float observer_field_theta,
    uint32_t seq) :
    team_id_(team_id),
    robot_id_(robot_id),
    ball_field_x_(ball_field_x),
    ball_field_y_(ball_field_y),
    confidence_(confidence),
    observer_field_x_(observer_field_x),
    observer_field_y_(observer_field_y),
    observer_field_theta_(observer_field_theta),
    seq_(seq) { }

  const std::string& team_id() const { return this->team_id_; }
  std::string& team_id() { return this->team_id_; }
  void team_id(const std::string& _val_) { this->team_id_ = _val_; }
  void team_id(std::string&& _val_) { this->team_id_ = _val_; }
  const std::string& robot_id() const { return this->robot_id_; }
  std::string& robot_id() { return this->robot_id_; }
  void robot_id(const std::string& _val_) { this->robot_id_ = _val_; }
  void robot_id(std::string&& _val_) { this->robot_id_ = _val_; }
  float ball_field_x() const { return this->ball_field_x_; }
  float& ball_field_x() { return this->ball_field_x_; }
  void ball_field_x(float _val_) { this->ball_field_x_ = _val_; }
  float ball_field_y() const { return this->ball_field_y_; }
  float& ball_field_y() { return this->ball_field_y_; }
  void ball_field_y(float _val_) { this->ball_field_y_ = _val_; }
  float confidence() const { return this->confidence_; }
  float& confidence() { return this->confidence_; }
  void confidence(float _val_) { this->confidence_ = _val_; }
  float observer_field_x() const { return this->observer_field_x_; }
  float& observer_field_x() { return this->observer_field_x_; }
  void observer_field_x(float _val_) { this->observer_field_x_ = _val_; }
  float observer_field_y() const { return this->observer_field_y_; }
  float& observer_field_y() { return this->observer_field_y_; }
  void observer_field_y(float _val_) { this->observer_field_y_ = _val_; }
  float observer_field_theta() const { return this->observer_field_theta_; }
  float& observer_field_theta() { return this->observer_field_theta_; }
  void observer_field_theta(float _val_) { this->observer_field_theta_ = _val_; }
  uint32_t seq() const { return this->seq_; }
  uint32_t& seq() { return this->seq_; }
  void seq(uint32_t _val_) { this->seq_ = _val_; }

  bool operator==(const TeamBall& _other) const
  {
    (void) _other;
    return team_id_ == _other.team_id_ &&
      robot_id_ == _other.robot_id_ &&
      ball_field_x_ == _other.ball_field_x_ &&
      ball_field_y_ == _other.ball_field_y_ &&
      confidence_ == _other.confidence_ &&
      observer_field_x_ == _other.observer_field_x_ &&
      observer_field_y_ == _other.observer_field_y_ &&
      observer_field_theta_ == _other.observer_field_theta_ &&
      seq_ == _other.seq_;
  }

  bool operator!=(const TeamBall& _other) const
  {
    return !(*this == _other);
  }
};

}

#include "dds/topic/TopicTraits.hpp"
#include "org/eclipse/cyclonedds/topic/datatopic.hpp"

namespace org {
namespace eclipse {
namespace cyclonedds {
namespace topic {

template <> constexpr const char* TopicTraits<::TeamBallModule::TeamBall>::getTypeName()
{
  return "TeamBallModule::TeamBall";
}

template <> constexpr bool TopicTraits<::TeamBallModule::TeamBall>::isKeyless()
{
  return true;
}

} //namespace topic
} //namespace cyclonedds
} //namespace eclipse
} //namespace org

namespace dds {
namespace topic {

template <>
struct topic_type_name<::TeamBallModule::TeamBall>
{
    static std::string value()
    {
      return org::eclipse::cyclonedds::topic::TopicTraits<::TeamBallModule::TeamBall>::getTypeName();
    }
};

}
}

REGISTER_TOPIC_TYPE(::TeamBallModule::TeamBall)

namespace org{
namespace eclipse{
namespace cyclonedds{
namespace core{
namespace cdr{

template<>
propvec &get_type_props<::TeamBallModule::TeamBall>();

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool write(T& streamer, const ::TeamBallModule::TeamBall& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!write_string(streamer, instance.team_id(), 0))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!write_string(streamer, instance.robot_id(), 0))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 2:
      if (!streamer.start_member(*prop))
        return false;
      if (!write(streamer, instance.ball_field_x()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 3:
      if (!streamer.start_member(*prop))
        return false;
      if (!write(streamer, instance.ball_field_y()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 4:
      if (!streamer.start_member(*prop))
        return false;
      if (!write(streamer, instance.confidence()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 5:
      if (!streamer.start_member(*prop))
        return false;
      if (!write(streamer, instance.observer_field_x()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 6:
      if (!streamer.start_member(*prop))
        return false;
      if (!write(streamer, instance.observer_field_y()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 7:
      if (!streamer.start_member(*prop))
        return false;
      if (!write(streamer, instance.observer_field_theta()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 8:
      if (!streamer.start_member(*prop))
        return false;
      if (!write(streamer, instance.seq()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool write(S& str, const ::TeamBallModule::TeamBall& instance, bool as_key) {
  auto &props = get_type_props<::TeamBallModule::TeamBall>();
  str.set_mode(cdr_stream::stream_mode::write, as_key);
  return write(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool read(T& streamer, ::TeamBallModule::TeamBall& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!read_string(streamer, instance.team_id(), 0))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!read_string(streamer, instance.robot_id(), 0))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 2:
      if (!streamer.start_member(*prop))
        return false;
      if (!read(streamer, instance.ball_field_x()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 3:
      if (!streamer.start_member(*prop))
        return false;
      if (!read(streamer, instance.ball_field_y()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 4:
      if (!streamer.start_member(*prop))
        return false;
      if (!read(streamer, instance.confidence()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 5:
      if (!streamer.start_member(*prop))
        return false;
      if (!read(streamer, instance.observer_field_x()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 6:
      if (!streamer.start_member(*prop))
        return false;
      if (!read(streamer, instance.observer_field_y()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 7:
      if (!streamer.start_member(*prop))
        return false;
      if (!read(streamer, instance.observer_field_theta()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 8:
      if (!streamer.start_member(*prop))
        return false;
      if (!read(streamer, instance.seq()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool read(S& str, ::TeamBallModule::TeamBall& instance, bool as_key) {
  auto &props = get_type_props<::TeamBallModule::TeamBall>();
  str.set_mode(cdr_stream::stream_mode::read, as_key);
  return read(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool move(T& streamer, const ::TeamBallModule::TeamBall& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!move_string(streamer, instance.team_id(), 0))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!move_string(streamer, instance.robot_id(), 0))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 2:
      if (!streamer.start_member(*prop))
        return false;
      if (!move(streamer, instance.ball_field_x()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 3:
      if (!streamer.start_member(*prop))
        return false;
      if (!move(streamer, instance.ball_field_y()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 4:
      if (!streamer.start_member(*prop))
        return false;
      if (!move(streamer, instance.confidence()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 5:
      if (!streamer.start_member(*prop))
        return false;
      if (!move(streamer, instance.observer_field_x()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 6:
      if (!streamer.start_member(*prop))
        return false;
      if (!move(streamer, instance.observer_field_y()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 7:
      if (!streamer.start_member(*prop))
        return false;
      if (!move(streamer, instance.observer_field_theta()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 8:
      if (!streamer.start_member(*prop))
        return false;
      if (!move(streamer, instance.seq()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool move(S& str, const ::TeamBallModule::TeamBall& instance, bool as_key) {
  auto &props = get_type_props<::TeamBallModule::TeamBall>();
  str.set_mode(cdr_stream::stream_mode::move, as_key);
  return move(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool max(T& streamer, const ::TeamBallModule::TeamBall& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!max_string(streamer, instance.team_id(), 0))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!max_string(streamer, instance.robot_id(), 0))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 2:
      if (!streamer.start_member(*prop))
        return false;
      if (!max(streamer, instance.ball_field_x()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 3:
      if (!streamer.start_member(*prop))
        return false;
      if (!max(streamer, instance.ball_field_y()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 4:
      if (!streamer.start_member(*prop))
        return false;
      if (!max(streamer, instance.confidence()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 5:
      if (!streamer.start_member(*prop))
        return false;
      if (!max(streamer, instance.observer_field_x()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 6:
      if (!streamer.start_member(*prop))
        return false;
      if (!max(streamer, instance.observer_field_y()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 7:
      if (!streamer.start_member(*prop))
        return false;
      if (!max(streamer, instance.observer_field_theta()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 8:
      if (!streamer.start_member(*prop))
        return false;
      if (!max(streamer, instance.seq()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool max(S& str, const ::TeamBallModule::TeamBall& instance, bool as_key) {
  auto &props = get_type_props<::TeamBallModule::TeamBall>();
  str.set_mode(cdr_stream::stream_mode::max, as_key);
  return max(str, instance, props.data()); 
}

} //namespace cdr
} //namespace core
} //namespace cyclonedds
} //namespace eclipse
} //namespace org

#endif // DDSCXX_TEAMBALLMODULE_HPP
