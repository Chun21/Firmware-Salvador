/***************************************************************
  TeamBallModule DDS type properties.
****************************************************************/
#include "common/TeamBallModule.hpp"

namespace org{
namespace eclipse{
namespace cyclonedds{
namespace core{
namespace cdr{

template<>
propvec &get_type_props<::TeamBallModule::TeamBall>() {
  static thread_local std::mutex mtx;
  static thread_local propvec props;
  static thread_local entity_properties_t *props_end = nullptr;
  static thread_local std::atomic_bool initialized {false};
  key_endpoint keylist;
  if (initialized.load(std::memory_order_relaxed)) {
    auto ptr = props.data();
    while (ptr < props_end)
      (ptr++)->is_present = false;
    return props;
  }
  std::lock_guard<std::mutex> lock(mtx);
  if (initialized.load(std::memory_order_relaxed)) {
    auto ptr = props.data();
    while (ptr < props_end)
      (ptr++)->is_present = false;
    return props;
  }
  props.clear();

  props.push_back(entity_properties_t(0, 0, false, bb_unset, extensibility::ext_final));  //root
  props.push_back(entity_properties_t(1, 0, false, bb_unset, extensibility::ext_final, false));  //::team_id
  props.push_back(entity_properties_t(1, 1, false, bb_unset, extensibility::ext_final, false));  //::robot_id
  props.push_back(entity_properties_t(1, 2, false, get_bit_bound<float>(), extensibility::ext_final, false));  //::ball_field_x
  props.push_back(entity_properties_t(1, 3, false, get_bit_bound<float>(), extensibility::ext_final, false));  //::ball_field_y
  props.push_back(entity_properties_t(1, 4, false, get_bit_bound<float>(), extensibility::ext_final, false));  //::confidence
  props.push_back(entity_properties_t(1, 5, false, get_bit_bound<float>(), extensibility::ext_final, false));  //::observer_field_x
  props.push_back(entity_properties_t(1, 6, false, get_bit_bound<float>(), extensibility::ext_final, false));  //::observer_field_y
  props.push_back(entity_properties_t(1, 7, false, get_bit_bound<float>(), extensibility::ext_final, false));  //::observer_field_theta
  props.push_back(entity_properties_t(1, 8, false, get_bit_bound<uint32_t>(), extensibility::ext_final, false));  //::seq

  entity_properties_t::finish(props, keylist);
  props_end = props.data() + props.size();
  initialized.store(true, std::memory_order_release);
  return props;
}

} //namespace cdr
} //namespace core
} //namespace cyclonedds
} //namespace eclipse
} //namespace org
