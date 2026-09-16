#include "fp2_component.h"
#include "stock_replay.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <cstdint>
#include <vector>
#include <span>
#include <cmath>
#include "esphome/core/entity_base.h"

namespace esphome {
namespace aqara_fp2 {

// CRC16-MODBUS
static uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void FP2Component::setup() {
  ESP_LOGI(TAG, "Setting up Aqara FP2...");

  load_settings_();
  load_zone_overrides_();
  load_map_overrides_();

  // Reset internal state
  waiting_for_ack_attr_id_ = AttrId::INVALID;
  init_done_ = false;

  // GPIO Reset
  perform_reset_();
}

void FP2Component::perform_reset_() {
  if (reset_pin_ != nullptr) {
    ESP_LOGI(TAG, "Performing Hardware Reset via Pin...");
    reset_pin_->setup();
    reset_pin_->digital_write(false);
    delay(100);
    reset_pin_->digital_write(true);
    ESP_LOGI(TAG, "Hardware Reset Done. Waiting for heartbeat...");
  } else {
    ESP_LOGI(TAG, "No Reset Pin configured. Waiting for heartbeat...");
  }

}

void FP2Component::set_location_reporting_enabled(bool enabled) {
  this->location_reporting_active_ = enabled;
  // 0x0112 is a BOOL attribute (type 0x04) - the radar ignores a UINT8 write
  this->enqueue_command_(OpCode::WRITE, AttrId::LOCATION_REPORT_ENABLE, (bool) enabled);
  if (!enabled && this->target_tracking_sensor_ != nullptr) {
    // Clear the sensor state when location reporting is disabled
    this->target_tracking_sensor_->set_has_state(false);
  }
}

void FP2LocationSwitch::write_state(bool state) {
  if (this->parent_ != nullptr) {
    this->parent_->set_location_reporting_enabled(state);
  }
  this->publish_state(state);
}

void FP2LocationSwitch::setup() {
  auto restored = this->get_initial_state_with_restore_mode();
  if (restored.has_value()) this->publish_state(*restored);
}

void FP2Component::loop() {
  while (available()) {
    uint8_t byte;
    read_byte(&byte);
    handle_incoming_byte_(byte);
  }

  check_initialization_();
  check_heartbeat_watchdog_();
  process_command_queue_();
  check_derived_absence_();

  if (millis() - rx_stats_last_ms_ > 60000) {
    rx_stats_last_ms_ = millis();
    ESP_LOGD(TAG, "RX stats: bytes=%u frames=%u dropped_sync=%u ver_mismatch=%u hdr_fail=%u crc_fail=%u",
             rx_bytes_, rx_frames_, rx_dropped_sync_, rx_ver_mismatch_, rx_hdr_fail_, rx_crc_fail_);
  }
}

void FP2Component::check_heartbeat_watchdog_() {
  if (!init_done_ || last_heartbeat_millis_ == 0)
    return;
  if (millis() - last_heartbeat_millis_ > HEARTBEAT_TIMEOUT_MS) {
    restart_initialization_("heartbeat lost");
  }
}

// Drop everything, reset the radar and let check_initialization_() run again
// on the next heartbeat.  Used when the heartbeat stops or when the radar
// announces a fresh boot (work_mode report) after we already configured it.
void FP2Component::restart_initialization_(const char *reason) {
  ESP_LOGW(TAG, "Radar re-initialization (%s)", reason);
  command_queue_.clear();
  waiting_for_ack_attr_id_ = AttrId::INVALID;
  init_done_ = false;
  last_heartbeat_millis_ = 0;
  perform_reset_();
}

void FP2Component::check_initialization_() {
  if (init_done_)
    return;

  // We rely on handle_parsed_frame_ to set a flag or we check
  // last_heartbeat_millis_
  if (last_heartbeat_millis_ > 0) {
    ESP_LOGI(TAG, "Heartbeat received. Starting initialization sequence...");
    init_done_ = true;

    init_time_millis_ = millis();

    // 0. Like the stock firmware: tell the radar how it is mounted and read
    //    its temperature before configuring anything.
    send_orientation_reports_();
    enqueue_read_(AttrId::TEMPERATURE);

    if (replay_stock_init_) {
      ESP_LOGW(TAG, "DEBUG: replaying %d stock boot writes verbatim", (int) STOCK_REPLAY_WRITES.size());
      for (const auto &payload : STOCK_REPLAY_WRITES) {
        FP2Command cmd;
        cmd.type = OpCode::WRITE;
        cmd.attr_id = (AttrId)((payload[0] << 8) | payload[1]);
        cmd.retry_count = 0;
        cmd.data = payload;
        command_queue_.push_back(cmd);
      }
      return;
    }

    // 1. Basic Settings
    enqueue_command_(OpCode::WRITE, AttrId::MONITOR_MODE, detection_direction_);
    enqueue_command_(OpCode::WRITE, AttrId::LEFT_RIGHT_REVERSE,
                     (uint8_t)(left_right_reverse_ ? 2 : 0));
    enqueue_command_(OpCode::WRITE, AttrId::PRESENCE_DETECT_SENSITIVITY, global_presence_sensitivity_);
    enqueue_command_(OpCode::WRITE, AttrId::CLOSING_SETTING, proximity_);
    enqueue_command_(OpCode::WRITE, AttrId::ZONE_CLOSE_AWAY_ENABLE, (uint16_t) 0x0001);
    // Stock always writes the fall sensitivity, even with fall detection off
    enqueue_command_(OpCode::WRITE, AttrId::FALL_SENSITIVITY, fall_detection_sensitivity_);
    if (fall_detection_) {
      enqueue_command_(OpCode::WRITE, AttrId::FALL_DETECT_ENABLE, (uint8_t) 1);
    }
    enqueue_command_(OpCode::WRITE, AttrId::PEOPLE_COUNT_REPORT_ENABLE, people_counting_); // BOOL
    enqueue_command_(OpCode::WRITE, AttrId::PEOPLE_NUMBER_ENABLE, people_counting_); // BOOL
    enqueue_command_(OpCode::WRITE, AttrId::TARGET_TYPE_ENABLE, ai_person_detection_); // BOOL
    // Sleep-zone geometry is written by stock at every boot (mount position 1,
    // 120 x 180 cm bed) even when sleep monitoring is off.
    {
      enqueue_command_(OpCode::WRITE, AttrId::SLEEP_MOUNT_POSITION, sleep_mount_position_);
      uint32_t size = ((uint32_t) sleep_bed_width_ << 16) | sleep_bed_length_;
      FP2Command cmd;
      cmd.type = OpCode::WRITE;
      cmd.attr_id = AttrId::SLEEP_ZONE_SIZE;
      cmd.retry_count = 0;
      cmd.data = {0x01, 0x69, 0x02, (uint8_t)(size >> 24), (uint8_t)(size >> 16), (uint8_t)(size >> 8), (uint8_t) size};
      command_queue_.push_back(cmd);
    }
    enqueue_command_(OpCode::WRITE, AttrId::WALL_CORNER_POS, mounting_position_);
    enqueue_command_(OpCode::WRITE, AttrId::DWELL_TIME_ENABLE, (uint8_t) 0);     // UINT8 in stock trace
    enqueue_command_(OpCode::WRITE, AttrId::WALK_DISTANCE_ENABLE, (uint8_t) 0);  // UINT8 in stock trace
    // NOTE: do not enable the thermodynamic chart (0x0138/0x0141); the stock
    // firmware never does, and radar FW 99 stops tracking when it is on.

    // 2. Grids (stock always writes an interference map, all-zero if none)
    if (has_interference_grid_) {
      // 0x0110 Interference Source
      enqueue_command_blob2_(AttrId::INTERFERENCE_MAP,
                             std::vector<uint8_t>(interference_grid_.begin(),
                                                  interference_grid_.end()));
    } else {
      enqueue_command_blob2_(AttrId::INTERFERENCE_MAP, std::vector<uint8_t>(40, 0x00));
    }
    if (has_exit_grid_) {
      // 0x0109 Enter/Exit Label
      enqueue_command_blob2_(
          AttrId::ENTRY_EXIT_MAP, std::vector<uint8_t>(exit_grid_.begin(), exit_grid_.end()));
    }
    if (has_edge_grid_) {
      // 0x0107 Edge Label
      enqueue_command_blob2_(
          AttrId::EDGE_MAP, std::vector<uint8_t>(edge_grid_.begin(), edge_grid_.end()));
    } else {
      // The radar keeps the last edge map in its own flash (e.g. the room the
      // Aqara app learned) and ignores targets in cells marked 1 (= edge /
      // blocked). Always write one: all-zero = nothing blocked.
      enqueue_command_blob2_(AttrId::EDGE_MAP, std::vector<uint8_t>(40, 0x00));
    }

    // 3. Zones
    for (const auto &zone : zones_) {
      send_zone_to_radar_(zone);
    }
    send_zone_activation_list_();
    for (const auto &zone : zones_) {
      // Zone type (0x0152): stock writes one per zone; 0x0a = "Others"
      enqueue_command_(OpCode::WRITE, AttrId::DETECT_ZONE_TYPE, (uint16_t)((zone->id << 8) | 0x0a));
      enqueue_command_(OpCode::WRITE, AttrId::ZONE_CLOSE_AWAY_ENABLE, (uint16_t)((zone->id << 8) | 1));
    }

    // Sleep monitoring is a radar MODE: while enabled the radar stops normal
    // presence/target reporting, and it remembers the setting across resets.
    // Always write it explicitly so a stale TRUE cannot linger.
    enqueue_command_(OpCode::WRITE, AttrId::SLEEP_REPORT_ENABLE, sleep_enabled_);

    // enqueue_read_((AttrId) 0x302); // Read radar flash ID attribute
    // enqueue_read_((AttrId) 0x303); // Read radar ID attribute
    // enqueue_read_((AttrId) 0x305); // Read radar calibration result attribute

    // 5. Publish grid sensors once initialization completes
    ESP_LOGI(TAG, "Publishing grid sensors: has_edge=%d edge_sensor=%p has_exit=%d exit_sensor=%p has_interference=%d interference_sensor=%p",
             has_edge_grid_, edge_label_grid_sensor_, has_exit_grid_, entry_exit_grid_sensor_, has_interference_grid_, interference_grid_sensor_);

    if (has_edge_grid_ && edge_label_grid_sensor_ != nullptr) {
      ESP_LOGI(TAG, "Publishing edge label grid");
      edge_label_grid_sensor_->publish_state(grid_to_hex_card_format(edge_grid_));
    } else {
      ESP_LOGW(TAG, "NOT publishing edge label grid (has_grid=%d, sensor=%p)", has_edge_grid_, edge_label_grid_sensor_);
    }

    if (has_exit_grid_ && entry_exit_grid_sensor_ != nullptr) {
      ESP_LOGI(TAG, "Publishing entry/exit grid");
      entry_exit_grid_sensor_->publish_state(grid_to_hex_card_format(exit_grid_));
    } else {
      ESP_LOGW(TAG, "NOT publishing entry/exit grid (has_grid=%d, sensor=%p)", has_exit_grid_, entry_exit_grid_sensor_);
    }

    if (has_interference_grid_ && interference_grid_sensor_ != nullptr) {
      ESP_LOGI(TAG, "Publishing interference grid");
      interference_grid_sensor_->publish_state(grid_to_hex_card_format(interference_grid_));
    } else {
      ESP_LOGW(TAG, "NOT publishing interference grid (has_grid=%d, sensor=%p)", has_interference_grid_, interference_grid_sensor_);
    }

    // 6. Publish zone map sensors
    for (const auto &zone : zones_) {
      publish_zone_map_(zone);
    }

    // 7. Publish known initial states after reset
    // After radar reset, we know there is no occupancy/motion detected yet
    ESP_LOGI(TAG, "Publishing initial zone states (no presence/motion after reset)");
    for (const auto &zone : zones_) {
      zone->publish_presence(false);
      zone->publish_motion(false);
    }

    if (global_presence_sensor_ != nullptr) global_presence_sensor_->publish_state(false);
    if (global_motion_sensor_ != nullptr) global_motion_sensor_->publish_state(false);

    // Clear target tracking state - no targets after reset
    if (target_tracking_sensor_ != nullptr) {
      target_tracking_sensor_->set_has_state(false);
    }

    // Re-apply the (possibly restored) location reporting switch
    if (location_report_switch_ != nullptr && location_report_switch_->state) {
      set_location_reporting_enabled(true);
    }

    publish_settings_();
  }
}

// ---- Runtime settings ----------------------------------------------------

static const char *const MOUNT_OPTS[] = {"wall", "left_corner", "right_corner"};
static const char *const PROX_OPTS[] = {"far", "medium", "close"};
static const char *const DIR_OPTS[] = {"default", "left_right"};
static const char *const SENS_OPTS[] = {"low", "medium", "high"};

static const char *setting_option_name(Setting setting, uint8_t value) {
  switch (setting) {
    case Setting::MOUNTING_POSITION: return (value >= 1 && value <= 3) ? MOUNT_OPTS[value - 1] : "wall";
    case Setting::PROXIMITY: return value <= 2 ? PROX_OPTS[value] : "medium";
    case Setting::DETECTION_DIRECTION: return value <= 1 ? DIR_OPTS[value] : "default";
    case Setting::SENSITIVITY:
    case Setting::FALL_SENSITIVITY: return (value >= 1 && value <= 3) ? SENS_OPTS[value - 1] : "medium";
    default: return "";
  }
}

static int setting_option_value(Setting setting, const std::string &name) {
  const char *const *opts; int n, base;
  switch (setting) {
    case Setting::MOUNTING_POSITION: opts = MOUNT_OPTS; n = 3; base = 1; break;
    case Setting::PROXIMITY: opts = PROX_OPTS; n = 3; base = 0; break;
    case Setting::DETECTION_DIRECTION: opts = DIR_OPTS; n = 2; base = 0; break;
    case Setting::SENSITIVITY:
    case Setting::FALL_SENSITIVITY: opts = SENS_OPTS; n = 3; base = 1; break;
    default: return -1;
  }
  for (int i = 0; i < n; i++) if (name == opts[i]) return base + i;
  return -1;
}

void FP2SettingSelect::control(const std::string &value) {
  int v = setting_option_value(setting_, value);
  if (v < 0 || parent_ == nullptr) return;
  parent_->apply_setting(setting_, (uint8_t) v);
  this->publish_state(value);
}

void FP2SettingSwitch::write_state(bool state) {
  if (parent_ != nullptr) parent_->apply_setting(setting_, state ? 1 : 0);
  this->publish_state(state);
}

uint8_t FP2Component::get_setting_value(Setting setting) const {
  switch (setting) {
    case Setting::MOUNTING_POSITION: return mounting_position_;
    case Setting::PROXIMITY: return proximity_;
    case Setting::DETECTION_DIRECTION: return detection_direction_;
    case Setting::SENSITIVITY: return global_presence_sensitivity_;
    case Setting::FALL_SENSITIVITY: return fall_detection_sensitivity_;
    case Setting::LEFT_RIGHT_REVERSE: return left_right_reverse_;
    case Setting::AI_PERSON_DETECTION: return ai_person_detection_;
    case Setting::PEOPLE_COUNTING: return people_counting_;
    case Setting::FALL_DETECTION: return fall_detection_;
    case Setting::SLEEP_MONITORING: return sleep_enabled_;
  }
  return 0;
}

void FP2Component::apply_setting(Setting setting, uint8_t value) {
  ESP_LOGI(TAG, "Setting %u -> %u", (unsigned) setting, value);
  switch (setting) {
    case Setting::MOUNTING_POSITION:
      mounting_position_ = value;
      if (init_done_) enqueue_command_(OpCode::WRITE, AttrId::WALL_CORNER_POS, mounting_position_);
      if (mounting_position_sensor_ != nullptr) mounting_position_sensor_->publish_state(get_mounting_position_string_());
      break;
    case Setting::PROXIMITY:
      proximity_ = value;
      if (init_done_) enqueue_command_(OpCode::WRITE, AttrId::CLOSING_SETTING, proximity_);
      break;
    case Setting::DETECTION_DIRECTION:
      detection_direction_ = value;
      if (init_done_) enqueue_command_(OpCode::WRITE, AttrId::MONITOR_MODE, detection_direction_);
      break;
    case Setting::SENSITIVITY:
      global_presence_sensitivity_ = value;
      if (init_done_) enqueue_command_(OpCode::WRITE, AttrId::PRESENCE_DETECT_SENSITIVITY, global_presence_sensitivity_);
      break;
    case Setting::FALL_SENSITIVITY:
      fall_detection_sensitivity_ = value;
      if (init_done_ && fall_detection_) enqueue_command_(OpCode::WRITE, AttrId::FALL_SENSITIVITY, fall_detection_sensitivity_);
      break;
    case Setting::LEFT_RIGHT_REVERSE:
      left_right_reverse_ = value != 0;
      if (init_done_) enqueue_command_(OpCode::WRITE, AttrId::LEFT_RIGHT_REVERSE, (uint8_t)(left_right_reverse_ ? 2 : 0));
      break;
    case Setting::AI_PERSON_DETECTION:
      ai_person_detection_ = value != 0;
      if (init_done_) enqueue_command_(OpCode::WRITE, AttrId::TARGET_TYPE_ENABLE, ai_person_detection_);
      break;
    case Setting::PEOPLE_COUNTING:
      people_counting_ = value != 0;
      if (init_done_) {
        enqueue_command_(OpCode::WRITE, AttrId::PEOPLE_COUNT_REPORT_ENABLE, people_counting_);
        enqueue_command_(OpCode::WRITE, AttrId::PEOPLE_NUMBER_ENABLE, people_counting_);
      }
      break;
    case Setting::FALL_DETECTION:
      fall_detection_ = value != 0;
      if (init_done_) {
        enqueue_command_(OpCode::WRITE, AttrId::FALL_DETECT_ENABLE, (uint8_t)(fall_detection_ ? 1 : 0));
        if (fall_detection_) enqueue_command_(OpCode::WRITE, AttrId::FALL_SENSITIVITY, fall_detection_sensitivity_);
      }
      break;
    case Setting::SLEEP_MONITORING:
      sleep_enabled_ = value != 0;
      if (init_done_) {
        if (sleep_enabled_) {
          enqueue_command_(OpCode::WRITE, AttrId::SLEEP_MOUNT_POSITION, sleep_mount_position_);
          uint32_t size = ((uint32_t) sleep_bed_width_ << 16) | sleep_bed_length_;
          FP2Command cmd;
          cmd.type = OpCode::WRITE; cmd.attr_id = AttrId::SLEEP_ZONE_SIZE; cmd.retry_count = 0;
          cmd.data = {0x01, 0x69, 0x02, (uint8_t)(size >> 24), (uint8_t)(size >> 16), (uint8_t)(size >> 8), (uint8_t) size};
          command_queue_.push_back(cmd);
        }
        enqueue_command_(OpCode::WRITE, AttrId::SLEEP_REPORT_ENABLE, sleep_enabled_);
        if (!sleep_enabled_ && sleep_presence_sensor_ != nullptr) sleep_presence_sensor_->publish_state(false);
      }
      break;
  }
  save_settings_();
}

void FP2Component::load_settings_() {
  settings_pref_ = global_preferences->make_preference<RuntimeSettings>(fnv1_hash("aqara_fp2_settings"));
  RuntimeSettings st{};
  if (!settings_pref_.load(&st) || st.magic != SETTINGS_MAGIC) return;
  mounting_position_ = st.mounting_position;
  proximity_ = st.proximity;
  detection_direction_ = st.detection_direction;
  global_presence_sensitivity_ = st.sensitivity;
  fall_detection_sensitivity_ = st.fall_sensitivity;
  left_right_reverse_ = st.left_right_reverse;
  ai_person_detection_ = st.ai_person_detection;
  people_counting_ = st.people_counting;
  fall_detection_ = st.fall_detection;
  sleep_enabled_ = st.sleep_monitoring;
  ESP_LOGI(TAG, "Loaded runtime settings from flash");
}

void FP2Component::save_settings_() {
  RuntimeSettings st{};
  st.magic = SETTINGS_MAGIC;
  st.mounting_position = mounting_position_;
  st.proximity = proximity_;
  st.detection_direction = detection_direction_;
  st.sensitivity = global_presence_sensitivity_;
  st.fall_sensitivity = fall_detection_sensitivity_;
  st.left_right_reverse = left_right_reverse_;
  st.ai_person_detection = ai_person_detection_;
  st.people_counting = people_counting_;
  st.fall_detection = fall_detection_;
  st.sleep_monitoring = sleep_enabled_;
  if (!settings_pref_.save(&st)) ESP_LOGW(TAG, "Failed to save runtime settings");
}

void FP2Component::publish_settings_() {
  for (auto *sel : setting_selects_)
    sel->publish_state(setting_option_name(sel->get_setting(), get_setting_value(sel->get_setting())));
  for (auto *sw : setting_switches_)
    sw->publish_state(get_setting_value(sw->get_setting()) != 0);
}

void FP2Component::process_command_queue_() {
  uint32_t now = millis();

  // Retry / drop the in-flight WRITE if its ACK is late
  if (waiting_for_ack_attr_id_ != AttrId::INVALID && now - last_command_sent_millis_ > ACK_TIMEOUT_MS) {
    in_flight_.retry_count++;
    if (in_flight_.retry_count >= MAX_RETRIES) {
      ESP_LOGW(TAG, "Command 0x%04X timed out after %d retries. Dropping.", (uint16_t) in_flight_.attr_id, MAX_RETRIES);
      waiting_for_ack_attr_id_ = AttrId::INVALID;
    } else {
      ESP_LOGW(TAG, "Command 0x%04X timed out. Retrying (%d/%d)...", (uint16_t) in_flight_.attr_id,
               in_flight_.retry_count, MAX_RETRIES);
      transmit_(in_flight_);
      last_command_sent_millis_ = now;
    }
  }

  // Send everything that doesn't need an ACK right away (ACKs, reverse-read
  // responses, reports), and the next WRITE once the previous one is ACKed.
  while (!command_queue_.empty()) {
    FP2Command &cmd = command_queue_.front();
    if (cmd.type == OpCode::WRITE) {
      if (waiting_for_ack_attr_id_ != AttrId::INVALID)
        return;  // still waiting
      in_flight_ = cmd;
      command_queue_.pop_front();
      transmit_(in_flight_);
      waiting_for_ack_attr_id_ = in_flight_.attr_id;
      last_command_sent_millis_ = millis();
      return;
    }
    FP2Command c = cmd;
    command_queue_.pop_front();
    transmit_(c);
  }
}

void FP2Component::send_next_command_() { process_command_queue_(); }

void FP2Component::transmit_(FP2Command &cmd) {
  static uint8_t next_tx_seq = 0;

  // Build frame: [Sync][Ver][Ver][Seq][Op][Len][Len][Check][Payload][CRC][CRC]
  std::vector<uint8_t> frame;
  frame.push_back(0x55);  // Sync
  frame.push_back(0x00);  // Version High
  frame.push_back(0x01);  // Version Low
  frame.push_back(cmd.seq >= 0 ? (uint8_t) cmd.seq : next_tx_seq++);
  frame.push_back((uint8_t) cmd.type);
  uint16_t len = cmd.data.size();
  frame.push_back((len >> 8) & 0xFF);
  frame.push_back(len & 0xFF);

  // Header checksum: NOT((Sum(bytes 0-6) - 1))
  uint8_t sum = 0;
  for (int i = 0; i < 7; i++)
    sum += frame[i];
  frame.push_back((uint8_t)(~((sum - 1))));

  // Append payload
  frame.insert(frame.end(), cmd.data.begin(), cmd.data.end());

  // Append CRC16 (Little Endian)
  uint16_t crc = crc16(frame.data(), frame.size());
  frame.push_back(crc & 0xFF);
  frame.push_back((crc >> 8) & 0xFF);

  write_array(frame);
  ESP_LOGV(TAG, "TX op=%d attr=0x%04X seq=%u len=%u", (int) cmd.type, (uint16_t) cmd.attr_id, frame[3], len);
}

void FP2Component::send_ack_(AttrId attr_id, uint8_t seq) {
  FP2Command cmd;
  cmd.type = OpCode::ACK;
  cmd.attr_id = attr_id;
  cmd.retry_count = 0;
  cmd.last_send_time = 0;
  cmd.seq = seq;  // echo the radar's sequence number

  // ACK payload: [SubID 2 bytes] [DataType VOID]
  cmd.data.push_back((((uint16_t) attr_id) >> 8) & 0xFF);
  cmd.data.push_back(((uint16_t) attr_id) & 0xFF);
  cmd.data.push_back(0x03);  // DataType: VOID

  command_queue_.push_back(cmd);
}

void FP2Component::send_reverse_response_(AttrId attr_id, uint16_t word_val, uint8_t seq) {
  FP2Command cmd;
  cmd.type = OpCode::READ;  // Reverse Read Response uses READ opcode
  cmd.attr_id = attr_id;
  cmd.retry_count = 0;
  cmd.seq = seq;
  cmd.data.push_back((((uint16_t) attr_id) >> 8) & 0xFF);
  cmd.data.push_back(((uint16_t) attr_id) & 0xFF);
  cmd.data.push_back(0x01);  // UINT16
  cmd.data.push_back((word_val >> 8) & 0xFF);
  cmd.data.push_back(word_val & 0xFF);
  command_queue_.push_back(cmd);
}

// Stock firmware sends these as REPORTs (op 5) right after the radar boots.
void FP2Component::send_orientation_reports_() {
  uint8_t dir = current_direction_();
  uint16_t angle = current_angle_();
  ESP_LOGI(TAG, "Reporting orientation to radar: direction=%u angle=%u", dir, angle);
  FP2Command c1;
  c1.type = OpCode::REPORT; c1.attr_id = AttrId::DEVICE_DIRECTION; c1.retry_count = 0;
  c1.data = {0x01, 0x43, 0x00, dir};
  command_queue_.push_back(c1);
  FP2Command c2;
  c2.type = OpCode::REPORT; c2.attr_id = AttrId::ANGLE_SENSOR_DATA; c2.retry_count = 0;
  c2.data = {0x01, 0x20, 0x01, (uint8_t)(angle >> 8), (uint8_t) angle};
  command_queue_.push_back(c2);
}

void FP2Component::send_reverse_response_(AttrId attr_id, uint8_t byte_val, uint8_t seq) {
  FP2Command cmd;
  cmd.type = OpCode::READ;  // Reverse Read Response uses READ opcode
  cmd.attr_id = attr_id;
  cmd.retry_count = 0;
  cmd.seq = seq;

  // Payload: [SubID 2 bytes] [DataType UINT8] [Value 1 byte]
  cmd.data.push_back((((uint16_t) attr_id) >> 8) & 0xFF);
  cmd.data.push_back(((uint16_t) attr_id) & 0xFF);
  cmd.data.push_back(0x00);  // DataType: UINT8
  cmd.data.push_back(byte_val);

  command_queue_.push_back(cmd);
}

void FP2Component::handle_incoming_byte_(uint8_t byte) {
  rx_bytes_++;
  if (state_ == SYNC) {
    if (byte == 0x55) {
      state_ = VER_H;
      rx_payload_.clear();
      header_sum_ = byte; // Start sum
    } else {
      rx_dropped_sync_++;
    }
    return;
  }

  // Update sum for header fields (0..6)
  if (state_ < H_CHECK) {
    header_sum_ += byte;
  }

  switch (state_) {
  case VER_H:
    if (byte != 0x00) rx_ver_mismatch_++;
    state_ = (byte == 0x00) ? VER_L : SYNC;
    break;
  case VER_L:
    if (byte != 0x01) rx_ver_mismatch_++;
    state_ = (byte == 0x01) ? SEQ : SYNC;
    break;
  case SEQ:
    rx_seq_ = byte;
    state_ = OPCODE;
    break;
  case OPCODE:
    rx_opcode_ = byte;
    state_ = LEN_H;
    break;
  case LEN_H:
    rx_len_ = byte << 8;
    state_ = LEN_L;
    break;
  case LEN_L:
    rx_len_ |= byte;
    state_ = H_CHECK;
    break;

  case H_CHECK: {
    uint8_t expected = (uint8_t)(~((header_sum_ - 1)));
    if (byte != expected) {
      ESP_LOGW(TAG, "Header Checksum Fail: Exp %02X, Got %02X", expected, byte);
      rx_hdr_fail_++;
      state_ = SYNC;
    } else {
      if (rx_len_ > 4096) { // Sanity check, increased for potential BLOBs
        state_ = SYNC;
      } else if (rx_len_ == 0) {
        state_ = CRC_L;
      } else {
        state_ = PAYLOAD;
      }
    }
    break;
  }

  case PAYLOAD:
    rx_payload_.push_back(byte);
    if (rx_payload_.size() == rx_len_) {
      state_ = CRC_L;
    }
    break;

  case CRC_L:
    rx_crc_ = byte;
    state_ = CRC_H;
    break;

  case CRC_H: {
    rx_crc_ |= (byte << 8);
    // Validate CRC
    std::vector<uint8_t> frame;
    frame.push_back(0x55);
    frame.push_back(0x00);
    frame.push_back(0x01);
    frame.push_back(rx_seq_);
    frame.push_back(rx_opcode_);
    frame.push_back((rx_len_ >> 8) & 0xFF);
    frame.push_back(rx_len_ & 0xFF);
    frame.push_back((uint8_t)(~((header_sum_ - 1))));
    frame.insert(frame.end(), rx_payload_.begin(), rx_payload_.end());

    uint16_t calc = crc16(frame.data(), frame.size());
    if (calc == rx_crc_) {
      rx_frames_++;
      // Parse Payload
      // Note: rx_len_ >= 2 to allow Reverse Read Requests (RESPONSE with just SubID, no data)
      if (rx_len_ >= 2) {
        uint16_t attr_id_int = (rx_payload_[0] << 8) | rx_payload_[1];
        AttrId attr_id = (AttrId) attr_id_int;
        // DataType = rx_payload_[2] (if present)
        handle_parsed_frame_(rx_opcode_, attr_id, rx_payload_);
      }
    } else {
      ESP_LOGW(TAG, "CRC Fail: Exp %04X, Got %04X", calc, rx_crc_);
      rx_crc_fail_++;
    }
    state_ = SYNC;
    break;
  }

  default:
    state_ = SYNC;
  }
}

void FP2Component::handle_parsed_frame_(uint8_t type, AttrId attr_id,
                                        const std::vector<uint8_t> &payload) {
  OpCode op = (OpCode)type;
  ESP_LOGV(TAG, "RX op=%d attr=0x%04X len=%d", type, (uint16_t) attr_id, payload.size());

  switch (op) {
    case OpCode::ACK:
      handle_ack_(attr_id);
      break;
    case OpCode::REPORT:
      handle_report_(attr_id, payload);
      break;
    case OpCode::RESPONSE:
      handle_response_(attr_id, payload);
      break;
    default:
      ESP_LOGW(TAG, "Unhandled OpCode: %d", type);
      break;
  }
}

void FP2Component::handle_ack_(AttrId attr_id) {
  if (waiting_for_ack_attr_id_ == attr_id) {
    ESP_LOGD(TAG, "ACK Received for 0x%04X", (uint16_t) attr_id);
    waiting_for_ack_attr_id_ = AttrId::INVALID;
  } else {
    ESP_LOGW(TAG, "Unexpected ACK 0x%04X (Waiting for 0x%04X)", attr_id,
             (uint16_t) waiting_for_ack_attr_id_);
  }
}

static const char *presence_event_name(uint8_t v) {
  // Same enum Aqara uses for presence_event on the FP1/FP2 (0x0103 report).
  switch (v) {
    case 0: return "enter";
    case 1: return "leave";
    case 2: return "left_enter";
    case 3: return "right_leave";
    case 4: return "right_enter";
    case 5: return "left_leave";
    case 6: return "approach";
    case 7: return "away";
    default: return "unknown";
  }
}

void FP2Component::handle_report_(AttrId attr_id, const std::vector<uint8_t> &payload) {
  // Send ACK for all reports except heartbeat
  if (attr_id != AttrId::RADAR_SW_VERSION) {
    send_ack_(attr_id, rx_seq_);
  }

  // payload: [SubID 2][DataType 1][data...]
  const uint8_t dtype = payload.size() >= 3 ? payload[2] : 0xFF;
  const bool is_u8 = (dtype == 0x00 && payload.size() == 4);
  const bool is_u16 = (dtype == 0x01 && payload.size() == 5);
  const bool is_u32 = (dtype == 0x02 && payload.size() == 7);
  const uint8_t u8 = is_u8 ? payload[3] : 0;
  const uint16_t u16 = is_u16 ? (uint16_t)((payload[3] << 8) | payload[4]) : 0;
  const uint32_t u32 = is_u32 ? ((uint32_t) payload[3] << 24 | (uint32_t) payload[4] << 16 |
                                 (uint32_t) payload[5] << 8 | payload[6]) : 0;

  switch (attr_id) {
    case AttrId::RADAR_SW_VERSION: {  // Heartbeat (~1 Hz)
      last_heartbeat_millis_ = millis();
      if (is_u8 && radar_software_sensor_ != nullptr) {
        auto ver_str = std::to_string(u8);
        if (radar_software_sensor_->state != ver_str)
          radar_software_sensor_->publish_state(ver_str);
      }
      break;
    }

    case AttrId::WORK_MODE:
      if (is_u8) {
        ESP_LOGI(TAG, "Received work mode report: %u", u8);
        // The radar sends this once on boot.  Seeing it long after we
        // configured it means the radar restarted behind our back.
        if (init_done_ && millis() - init_time_millis_ > 10000) {
          restart_initialization_("radar rebooted");
        }
      }
      break;

    case AttrId::DETECT_ZONE_MOTION: {
      // UINT16: Hi=ZoneID, Lo=event bitmask (1=enter, 2=move, 4=exit, 8=left/right)
      if (!is_u16) break;
      uint8_t zone_id = u16 >> 8;
      uint8_t ev = u16 & 0xFF;
      ESP_LOGD(TAG, "Zone Motion Report: Zone %u = 0x%02X", zone_id, ev);
      for (auto &z : zones_) {
        if (z->id != zone_id) continue;
        if (ev & 0x04) {
          z->publish_motion(false);
          z->publish_event("exit");
        } else if (ev & 0x03) {
          z->publish_motion(true);
          z->publish_event((ev & 0x01) ? "enter" : "move");
        } else {
          char buf[8];
          snprintf(buf, sizeof(buf), "0x%02x", ev);
          z->publish_event(buf);
        }
        break;
      }
      break;
    }

    case AttrId::MOTION_DETECT:
      // Not a boolean: presence-event enum. Even values are arrivals
      // (enter/left_enter/right_enter/approach), odd are departures.
      if (is_u8) {
        ESP_LOGI(TAG, "Presence event: %u (%s)", u8, presence_event_name(u8));
        if (presence_event_sensor_ != nullptr)
          presence_event_sensor_->publish_state(presence_event_name(u8));
        if (global_motion_sensor_ != nullptr)
          global_motion_sensor_->publish_state((u8 & 0x01) == 0);
      }
      break;

    case AttrId::PRESENCE_DETECT:
      if (is_u8) {
        radar_presence_seen_ = true;
        ESP_LOGI(TAG, "Received global presence report: %u", u8);
        if (global_presence_sensor_ != nullptr)
          global_presence_sensor_->publish_state(u8 != 0);
      }
      break;

    case AttrId::REALTIME_PEOPLE_NUMBER:
      if (is_u32) {
        ESP_LOGD(TAG, "Realtime people number: %u", u32);
        if (people_count_sensor_ != nullptr)
          people_count_sensor_->publish_state(u32);
      }
      break;

    case AttrId::ONTIME_PEOPLE_NUMBER:
      if (is_u32) ESP_LOGD(TAG, "Ontime people number: %u", u32);
      break;

    case AttrId::PEOPLE_COUNTING:
      ESP_LOGD(TAG, "People counting blob (%d bytes)", payload.size());
      break;

    case AttrId::ZONE_PRESENCE: {
      // UINT16: Hi=ZoneID, Lo=State (1=Occupied, 0=Empty)
      if (!is_u16) break;
      uint8_t zone_id = u16 >> 8;
      uint8_t state = u16 & 0xFF;
      radar_zone_seen_ = true;
      ESP_LOGD(TAG, "Zone Presence Report: Zone %d = %s", zone_id, state ? "ON" : "OFF");
      for (auto &z : zones_) {
        if (z->id == zone_id) {
          z->publish_presence(state == 1);
          if (state == 0) z->publish_motion(false);
          break;
        }
      }
      break;
    }

    case AttrId::LOCATION_TRACKING_DATA:
      handle_location_tracking_report_(payload);
      break;

    case AttrId::TEMPERATURE:
      handle_temperature_report_(payload);
      break;

    case AttrId::SLEEP_DATA:
    case AttrId::SLEEP_STATE:
    case AttrId::SLEEP_PRESENCE:
    case AttrId::SLEEP_INOUT:
    case AttrId::SLEEP_EVENT:
      handle_sleep_report_(attr_id, payload);
      break;

    case AttrId::THERMO_DATA:
    case AttrId::TARGET_POSTURE:
      break;  // not used

    default:
      ESP_LOGW(TAG, "Unhandled report 0x%04X (%d bytes)", (uint16_t) attr_id, payload.size());
      break;
  }
}

// Sleep monitoring reports.  The enum meanings are not verified yet, so the
// raw values are published and logged at INFO so they can be correlated with
// what a person in the bed is actually doing.
void FP2Component::handle_sleep_report_(AttrId attr_id, const std::vector<uint8_t> &payload) {
  const uint8_t dtype = payload.size() >= 3 ? payload[2] : 0xFF;
  if (dtype == 0x00 && payload.size() == 4) {
    uint8_t v = payload[3];
    ESP_LOGI(TAG, "Sleep report 0x%04X = %u", (uint16_t) attr_id, v);
    switch (attr_id) {
      case AttrId::SLEEP_PRESENCE:
        if (sleep_presence_sensor_ != nullptr) sleep_presence_sensor_->publish_state(v != 0);
        break;
      case AttrId::SLEEP_STATE:
        if (sleep_state_sensor_ != nullptr) sleep_state_sensor_->publish_state(v);
        break;
      case AttrId::SLEEP_INOUT:
        if (sleep_inout_sensor_ != nullptr) sleep_inout_sensor_->publish_state(v);
        break;
      case AttrId::SLEEP_EVENT:
        if (sleep_event_sensor_ != nullptr) sleep_event_sensor_->publish_state(v);
        break;
      default:
        break;
    }
    return;
  }
  if (dtype == 0x06 && payload.size() >= 5) {
    // BLOB2: [len 2][count 1][12-byte items: target, zone, presence, 9 unknown]
    std::string hex;
    char b[4];
    for (size_t i = 5; i < payload.size(); i++) {
      snprintf(b, sizeof(b), "%02x", payload[i]);
      hex += b;
    }
    ESP_LOGI(TAG, "Sleep data: %s", hex.c_str());
    if (sleep_data_sensor_ != nullptr) sleep_data_sensor_->publish_state(hex);
    return;
  }
  ESP_LOGW(TAG, "Sleep report 0x%04X with unexpected format (%d bytes)", (uint16_t) attr_id, payload.size());
}

void FP2Component::handle_location_tracking_report_(const std::vector<uint8_t> &payload) {
  // Ignore stale data if location reporting has been disabled
  if (!this->location_reporting_active_) {
    return;
  }

  // Payload: [SubID 2] [Type 0x06(BLOB2)] [Len 2] [Count 1] [Target 14]...
  if (payload.size() < 6 || payload[2] != 0x06) {
    return;
  }

  uint8_t count = payload[5];

  // Build binary buffer: [count][target1 14 bytes][target2 14 bytes]...
  // Each target is 14 bytes: id(1), x(2), y(2), z(2), velocity(2), snr(2), classifier(1), posture(1), active(1)
  std::vector<uint8_t> binary_data;
  binary_data.push_back(count);

  for (int i = 0; i < count; i++) {
    int offset = 6 + (i * 14);
    if (offset + 14 > payload.size())
      break;

    // Copy raw 14-byte target data directly (already in correct big-endian format)
    binary_data.insert(binary_data.end(),
                       payload.begin() + offset,
                       payload.begin() + offset + 14);
  }

  if (derive_presence_)
    update_derived_states_(payload, count);

  // Base64 encode the binary data
  std::string base64_str = esphome::base64_encode(binary_data);

  if (this->target_tracking_sensor_ != nullptr) {
    this->target_tracking_sensor_->publish_state(base64_str);
  }
}

// ---- Derived presence ----------------------------------------------------

// Target coordinates are ~cm, radar at the top of the 14x14 grid (0.5 m cells).
// Corner modes: X +400 = left edge, -400 = right edge, Y 0..800 (hansihe).
// Wall mode: sensor top-centre, X negative = right.
bool FP2Component::zone_contains_(const FP2Zone *zone, int16_t x, int16_t y) const {
  int col, row;
  if (mounting_position_ == 0x02 || mounting_position_ == 0x03) {
    // corner: 14x14 view at columns 2-15, +X = left
    col = 2 + (int) ((-(float) x + 400.0f) / 800.0f * 14.0f);
    row = (int) ((float) y / 800.0f * 14.0f);
  } else {
    // wall: full 16x20 grid, sensor at column 8 / row 0, 50 cm cells of 20
    // units (2.5 cm per unit, measured), +X = left
    col = 8 - (int) floorf((float) x / 20.0f) - 1;
    row = (int) ((float) y / 20.0f);
  }
  if (col < 0 || col > 15 || row < 0 || row > 19) return false;
  uint16_t bits = (zone->grid[row * 2] << 8) | zone->grid[row * 2 + 1];
  return (bits >> (15 - col)) & 1;
}

void FP2Component::update_derived_states_(const std::vector<uint8_t> &payload, uint8_t count) {
  uint32_t now = millis();
  if (zone_last_seen_ms_.size() != zones_.size()) zone_last_seen_ms_.assign(zones_.size(), 0);
  bool moving = false;
  std::vector<bool> zone_hit(zones_.size(), false);
  for (int i = 0; i < count; i++) {
    int off = 6 + i * 14;
    if (off + 14 > (int) payload.size()) break;
    int16_t x = (int16_t)((payload[off + 1] << 8) | payload[off + 2]);
    int16_t y = (int16_t)((payload[off + 3] << 8) | payload[off + 4]);
    int16_t v = (int16_t)((payload[off + 7] << 8) | payload[off + 8]);
    if (v > motion_velocity_threshold_ || v < -motion_velocity_threshold_) moving = true;
    for (size_t z = 0; z < zones_.size(); z++)
      if (!zone_hit[z] && !zones_[z]->is_empty() && zone_contains_(zones_[z], x, y)) zone_hit[z] = true;
  }
  if (count > 0) {
    last_target_seen_ms_ = now;
    if (!radar_presence_seen_) {
      if (global_presence_sensor_ != nullptr && (!global_presence_sensor_->has_state() || !global_presence_sensor_->state))
        global_presence_sensor_->publish_state(true);
      if (global_motion_sensor_ != nullptr && (!global_motion_sensor_->has_state() || global_motion_sensor_->state != moving))
        global_motion_sensor_->publish_state(moving);
    }
  }
  if (!radar_zone_seen_) {
    for (size_t z = 0; z < zones_.size(); z++) {
      auto *zone = zones_[z];
      if (zone_hit[z]) {
        zone_last_seen_ms_[z] = now;
        if (zone->presence_sensor != nullptr && (!zone->presence_sensor->has_state() || !zone->presence_sensor->state))
          zone->publish_presence(true);
        if (zone->motion_sensor != nullptr && (!zone->motion_sensor->has_state() || zone->motion_sensor->state != moving))
          zone->publish_motion(moving);
      }
    }
  }
}

void FP2Component::check_derived_absence_() {
  if (!derive_presence_ || !init_done_) return;
  uint32_t now = millis();
  if (!radar_presence_seen_ && last_target_seen_ms_ != 0 && now - last_target_seen_ms_ > absence_timeout_ms_) {
    if (global_presence_sensor_ != nullptr && global_presence_sensor_->state) global_presence_sensor_->publish_state(false);
    if (global_motion_sensor_ != nullptr && global_motion_sensor_->state) global_motion_sensor_->publish_state(false);
  }
  if (!radar_zone_seen_) {
    for (size_t z = 0; z < zones_.size() && z < zone_last_seen_ms_.size(); z++) {
      auto *zone = zones_[z];
      if (zone_last_seen_ms_[z] != 0 && now - zone_last_seen_ms_[z] > absence_timeout_ms_) {
        if (zone->presence_sensor != nullptr && zone->presence_sensor->state) zone->publish_presence(false);
        if (zone->motion_sensor != nullptr && zone->motion_sensor->state) zone->publish_motion(false);
      }
    }
  }
}

void FP2Component::handle_temperature_report_(const std::vector<uint8_t> &payload) {
    if (payload.size() == 5 && payload[2] == 0x01) {
        uint16_t temp = payload[3] << 8 | payload[4];
        if (radar_temperature_sensor_ != nullptr) {
            radar_temperature_sensor_->publish_state(temp);
        }
        ESP_LOGD(TAG, "Radar temperature report: %d", temp);
    } else {
        ESP_LOGD(TAG, "Unexpected radar temperature report format");
    }
}

void FP2Component::handle_response_(AttrId attr_id, const std::vector<uint8_t> &payload) {
  // RESPONSE packets with only 2 bytes (just SubID) are Reverse Read Requests from the radar
  if (payload.size() == 2) {
    handle_reverse_read_request_(attr_id);
  } else {
    // Normal Response with data (currently unused)
    ESP_LOGD(TAG, "Received Response for 0x%04X with %d bytes", (uint16_t) attr_id, payload.size());
  }
}

void FP2Component::handle_reverse_read_request_(AttrId attr_id) {
  ESP_LOGI(TAG, "Received Reverse Query for SubID 0x%04X", (uint16_t) attr_id);

  switch (attr_id) {
    case AttrId::DEVICE_DIRECTION:  // device_direction
      send_reverse_response_(attr_id, current_direction_(), rx_seq_);
      ESP_LOGD(TAG, "Sending Device Direction: %d (seq %u)", current_direction_(), rx_seq_);
      break;

    case AttrId::ANGLE_SENSOR_DATA:  // angle_sensor_data (UINT16 in the stock traces)
      send_reverse_response_(attr_id, current_angle_(), rx_seq_);
      ESP_LOGD(TAG, "Sending Angle Sensor Data: %d", current_angle_());
      break;

    default:
      ESP_LOGW(TAG, "Unknown Reverse Query SubID 0x%04X", (uint16_t) attr_id);
      break;
  }
}

// Command Queue Helpers
void FP2Component::enqueue_command_(OpCode type, AttrId attr_id,
                                    uint8_t byte_val) {
  FP2Command cmd;
  cmd.type = type;
  cmd.attr_id = attr_id;
  cmd.retry_count = 0;

  // Payload: [SubID 2] [Type 1] [Data 1]
  cmd.data.push_back((((uint16_t) attr_id) >> 8) & 0xFF);
  cmd.data.push_back(((uint16_t) attr_id) & 0xFF);
  cmd.data.push_back(0x00); // UINT8
  cmd.data.push_back(byte_val);

  command_queue_.push_back(cmd);
}

void FP2Component::enqueue_command_(OpCode type, AttrId attr_id,
                                    uint16_t word_val) {
  FP2Command cmd;
  cmd.type = type;
  cmd.attr_id = attr_id;
  cmd.retry_count = 0;

  // Payload: [SubID 2] [Type 1] [Data 2]
  cmd.data.push_back((((uint16_t) attr_id) >> 8) & 0xFF);
  cmd.data.push_back(((uint16_t) attr_id) & 0xFF);
  cmd.data.push_back(0x01); // UINT16
  cmd.data.push_back((word_val >> 8) & 0xFF);
  cmd.data.push_back(word_val & 0xFF);

  command_queue_.push_back(cmd);
}

void FP2Component::enqueue_command_(OpCode type, AttrId attr_id,
                                    bool bool_val) {
  FP2Command cmd;
  cmd.type = type;
  cmd.attr_id = attr_id;
  cmd.retry_count = 0;

  // Payload: [SubID 2] [Type 1] [Data 1]
  cmd.data.push_back((((uint16_t) attr_id) >> 8) & 0xFF);
  cmd.data.push_back(((uint16_t) attr_id) & 0xFF);
  cmd.data.push_back(0x04); // BOOL
  cmd.data.push_back((uint8_t) bool_val);

  command_queue_.push_back(cmd);
}


void FP2Component::enqueue_command_blob2_(
    AttrId attr_id, const std::vector<uint8_t> &blob_content) {
  FP2Command cmd;
  cmd.type = OpCode::WRITE; // Always Write for these configs
  cmd.attr_id = attr_id;
  cmd.retry_count = 0;

  // Payload: [SubID 2] [Type 1 (0x06)] [Len 2] [Content N]
  cmd.data.push_back((((uint16_t) attr_id) >> 8) & 0xFF);
  cmd.data.push_back(((uint16_t) attr_id) & 0xFF);
  cmd.data.push_back(0x06); // BLOB2

  uint16_t len = blob_content.size();
  cmd.data.push_back((len >> 8) & 0xFF);
  cmd.data.push_back(len & 0xFF);

  cmd.data.insert(cmd.data.end(), blob_content.begin(), blob_content.end());

  command_queue_.push_back(cmd);
}

void FP2Component::enqueue_read_(AttrId attr_id) {
    FP2Command cmd;
    cmd.type = OpCode::RESPONSE;
    cmd.attr_id = attr_id;
    cmd.retry_count = 0;

    cmd.data.push_back((((uint16_t) attr_id) >> 8) & 0xFF);
    cmd.data.push_back(((uint16_t) attr_id) & 0xFF);

    command_queue_.push_back(cmd);
}

void FP2Component::set_interference_grid(const std::vector<uint8_t> &grid) {
  ESP_LOGI(TAG, "set_interference_grid called with size: %d", grid.size());
  if (grid.size() == 40) {
    std::copy(grid.begin(), grid.end(), interference_grid_.begin());
    yaml_interference_grid_ = interference_grid_; yaml_has_interference_ = true;
    has_interference_grid_ = true;
    ESP_LOGI(TAG, "Interference grid configured successfully");
  } else {
    ESP_LOGW(TAG, "Interference grid size mismatch! Expected 40, got %d", grid.size());
  }
}

void FP2Component::set_exit_grid(const std::vector<uint8_t> &grid) {
  ESP_LOGI(TAG, "set_exit_grid called with size: %d", grid.size());
  if (grid.size() == 40) {
    std::copy(grid.begin(), grid.end(), exit_grid_.begin());
    yaml_exit_grid_ = exit_grid_; yaml_has_exit_ = true;
    has_exit_grid_ = true;
    ESP_LOGI(TAG, "Exit grid configured successfully");
  } else {
    ESP_LOGW(TAG, "Exit grid size mismatch! Expected 40, got %d", grid.size());
  }
}

void FP2Component::set_edge_grid(const std::vector<uint8_t> &grid) {
  ESP_LOGI(TAG, "set_edge_grid called with size: %d", grid.size());
  if (grid.size() == 40) {
    std::copy(grid.begin(), grid.end(), edge_grid_.begin());
    yaml_edge_grid_ = edge_grid_; yaml_has_edge_ = true;
    has_edge_grid_ = true;
    ESP_LOGI(TAG, "Edge grid configured successfully");
  } else {
    ESP_LOGW(TAG, "Edge grid size mismatch! Expected 40, got %d", grid.size());
  }
}

void FP2Component::set_zones(const std::vector<FP2Zone*> &zones) {
    zones_ = zones;
}

// Full protocol grid: 20 rows x 16 columns, 4 hex chars per row (80 chars).
// Wall mode uses the whole grid (8 m x 10 m, 0.5 m cells, sensor top-centre);
// corner modes use rows 0-13 / columns 2-15 (the classic 14 x 14 view).
std::string FP2Component::grid_to_hex_card_format(const GridMap &grid) {
  std::string result;
  result.reserve(80);
  for (int R = 0; R < 20; R++) {
    char hex[5];
    snprintf(hex, sizeof(hex), "%02x%02x", grid[R * 2], grid[R * 2 + 1]);
    result += hex;
  }
  return result;
}

// void FP2Component::add_zone(uint8_t id, binary_sensor::BinarySensor *sens,
//                             const std::vector<uint8_t> &grid,
//                             uint8_t sensitivity) {
//   FP2Zone z;
//   z.id = id;
//   z.occupancy = sens;
//   z.sensitivity = sensitivity;
//
//   if (grid.size() == 40) {
//     std::copy(grid.begin(), grid.end(), z.grid.begin());
//     zones_.push_back(z);
//   } else {
//     ESP_LOGE(TAG, "Zone %d grid size mismatch: %d != 40", id, grid.size());
//   }
// }

// ---- Runtime zone editing ----------------------------------------------

void FP2Component::send_zone_to_radar_(FP2Zone *zone) {
  // 0x0114: [ZoneID][40 byte map]
  std::vector<uint8_t> payload;
  payload.push_back(zone->id);
  payload.insert(payload.end(), zone->grid.begin(), zone->grid.end());
  enqueue_command_blob2_(AttrId::ZONE_MAP, payload);
  // 0x0151: Hi=ZoneID, Lo=Sensitivity
  enqueue_command_(OpCode::WRITE, AttrId::ZONE_SENSITIVITY, (uint16_t)((zone->id << 8) | (zone->sensitivity & 0xFF)));
}

void FP2Component::send_zone_activation_list_() {
  std::vector<uint8_t> activations(32, 0);
  for (const auto &zone : zones_) {
    if (!zone->is_empty() && zone->id < activations.size())
      activations[zone->id] = zone->id;
  }
  enqueue_command_blob2_(AttrId::ZONE_ACTIVATION_LIST, activations);
}

void FP2Component::publish_zone_map_(FP2Zone *zone) {
  if (zone->map_sensor != nullptr)
    zone->map_sensor->publish_state(grid_to_hex_card_format(zone->grid));
}

void FP2Component::load_zone_overrides_() {
  zone_pref_ = global_preferences->make_preference<ZoneStore>(fnv1_hash("aqara_fp2_zones"));
  ZoneStore store{};
  if (!zone_pref_.load(&store) || store.magic != ZONE_STORE_MAGIC)
    return;
  for (size_t i = 0; i < zones_.size() && i < MAX_RUNTIME_ZONES; i++) {
    const auto &o = store.zones[i];
    if (!o.valid) continue;
    std::copy(o.grid, o.grid + 40, zones_[i]->grid.begin());
    zones_[i]->sensitivity = o.sensitivity;
    zones_[i]->runtime_override = true;
    ESP_LOGI(TAG, "Zone %u: loaded runtime override (sens %u)", zones_[i]->id, o.sensitivity);
  }
}

void FP2Component::save_zone_overrides_() {
  ZoneStore store{};
  store.magic = ZONE_STORE_MAGIC;
  for (size_t i = 0; i < zones_.size() && i < MAX_RUNTIME_ZONES; i++) {
    auto &o = store.zones[i];
    o.valid = zones_[i]->runtime_override ? 1 : 0;
    o.sensitivity = zones_[i]->sensitivity;
    std::copy(zones_[i]->grid.begin(), zones_[i]->grid.end(), o.grid);
  }
  if (!zone_pref_.save(&store))
    ESP_LOGW(TAG, "Failed to save zone overrides");
}

// ---- Runtime map editing (interference / edge / entry-exit) -------------

int FP2Component::map_kind_(const std::string &kind) const {
  if (kind == "interference") return 0;
  if (kind == "edge" || kind == "exclude") return 1;
  if (kind == "exit" || kind == "entry_exit" || kind == "entry") return 2;
  return -1;
}

void FP2Component::send_map_to_radar_(int kind) {
  if (!init_done_) return;
  switch (kind) {
    case 0: enqueue_command_blob2_(AttrId::INTERFERENCE_MAP, std::vector<uint8_t>(interference_grid_.begin(), interference_grid_.end())); break;
    case 1: enqueue_command_blob2_(AttrId::EDGE_MAP, std::vector<uint8_t>(edge_grid_.begin(), edge_grid_.end())); break;
    case 2: enqueue_command_blob2_(AttrId::ENTRY_EXIT_MAP, std::vector<uint8_t>(exit_grid_.begin(), exit_grid_.end())); break;
  }
}

void FP2Component::publish_map_sensor_(int kind) {
  switch (kind) {
    case 0: if (interference_grid_sensor_) interference_grid_sensor_->publish_state(grid_to_hex_card_format(interference_grid_)); break;
    case 1: if (edge_label_grid_sensor_) edge_label_grid_sensor_->publish_state(grid_to_hex_card_format(edge_grid_)); break;
    case 2: if (entry_exit_grid_sensor_) entry_exit_grid_sensor_->publish_state(grid_to_hex_card_format(exit_grid_)); break;
  }
}

void FP2Component::load_map_overrides_() {
  map_pref_ = global_preferences->make_preference<MapStore>(fnv1_hash("aqara_fp2_maps"));
  MapStore st{};
  if (!map_pref_.load(&st) || st.magic != MAP_STORE_MAGIC) return;
  GridMap *grids[3] = {&interference_grid_, &edge_grid_, &exit_grid_};
  bool *has[3] = {&has_interference_grid_, &has_edge_grid_, &has_exit_grid_};
  for (int k = 0; k < 3; k++) {
    if (!st.valid[k]) continue;
    std::copy(st.maps[k], st.maps[k] + 40, grids[k]->begin());
    *has[k] = true;
    map_override_[k] = true;
    ESP_LOGI(TAG, "Map %d: loaded runtime override", k);
  }
}

void FP2Component::save_map_overrides_() {
  MapStore st{};
  st.magic = MAP_STORE_MAGIC;
  const GridMap *grids[3] = {&interference_grid_, &edge_grid_, &exit_grid_};
  for (int k = 0; k < 3; k++) {
    st.valid[k] = map_override_[k] ? 1 : 0;
    std::copy(grids[k]->begin(), grids[k]->end(), st.maps[k]);
  }
  if (!map_pref_.save(&st)) ESP_LOGW(TAG, "Failed to save map overrides");
}

bool FP2Component::set_map_config(const std::string &kind, const std::string &grid_hex) {
  int k = map_kind_(kind);
  if (k < 0) { ESP_LOGW(TAG, "set_map: unknown kind '%s'", kind.c_str()); return false; }
  GridMap g;
  if (!hex_card_format_to_grid(grid_hex, g)) { ESP_LOGW(TAG, "set_map: bad grid"); return false; }
  GridMap *grids[3] = {&interference_grid_, &edge_grid_, &exit_grid_};
  bool *has[3] = {&has_interference_grid_, &has_edge_grid_, &has_exit_grid_};
  *grids[k] = g;
  *has[k] = true;
  map_override_[k] = true;
  save_map_overrides_();
  ESP_LOGI(TAG, "Map '%s' updated from API", kind.c_str());
  send_map_to_radar_(k);
  publish_map_sensor_(k);
  return true;
}

bool FP2Component::reset_map_config(const std::string &kind) {
  int k = map_kind_(kind);
  if (k < 0) return false;
  GridMap *grids[3] = {&interference_grid_, &edge_grid_, &exit_grid_};
  const GridMap *yaml[3] = {&yaml_interference_grid_, &yaml_edge_grid_, &yaml_exit_grid_};
  bool yaml_has[3] = {yaml_has_interference_, yaml_has_edge_, yaml_has_exit_};
  bool *has[3] = {&has_interference_grid_, &has_edge_grid_, &has_exit_grid_};
  if (yaml_has[k]) { *grids[k] = *yaml[k]; *has[k] = true; } else { grids[k]->fill(0); *has[k] = false; }
  map_override_[k] = false;
  save_map_overrides_();
  ESP_LOGI(TAG, "Map '%s' reset to YAML config", kind.c_str());
  if (init_done_) {
    // send the effective map (all-zero when none is configured)
    std::vector<uint8_t> v(grids[k]->begin(), grids[k]->end());
    if (k == 0) enqueue_command_blob2_(AttrId::INTERFERENCE_MAP, v);
    else if (k == 1) enqueue_command_blob2_(AttrId::EDGE_MAP, v);
    else enqueue_command_blob2_(AttrId::ENTRY_EXIT_MAP, v);
  }
  publish_map_sensor_(k);
  return true;
}

bool FP2Component::hex_card_format_to_grid(const std::string &hex, GridMap &grid) {
  // 20 rows (80 chars) or legacy 14 rows (56 chars) x 4 hex chars per row
  if (hex.size() != 80 && hex.size() != 56) return false;
  grid.fill(0);
  const int rows = hex.size() / 4;
  for (int r = 0; r < rows; r++) {
    unsigned int v;
    if (sscanf(hex.substr(r * 4, 4).c_str(), "%4x", &v) != 1) return false;
    grid[r * 2] = (v >> 8) & 0xFF;
    grid[r * 2 + 1] = v & 0xFF;
  }
  return true;
}

bool FP2Component::set_zone_config(int zone_id, const std::string &grid_hex, int sensitivity) {
  for (auto *zone : zones_) {
    if (zone->id != zone_id) continue;
    GridMap g;
    if (!hex_card_format_to_grid(grid_hex, g)) {
      ESP_LOGW(TAG, "set_zone: bad grid '%s' (need 80 or 56 hex chars)", grid_hex.c_str());
      return false;
    }
    if (sensitivity < 1 || sensitivity > 3) sensitivity = zone->sensitivity;
    zone->grid = g;
    zone->sensitivity = sensitivity;
    zone->runtime_override = true;
    save_zone_overrides_();
    ESP_LOGI(TAG, "Zone %u updated from API (sens %d)", zone->id, sensitivity);
    if (init_done_) {
      send_zone_to_radar_(zone);
      send_zone_activation_list_();
    }
    publish_zone_map_(zone);
    // A cleared zone can't be occupied
    if (zone->is_empty()) {
      zone->publish_presence(false);
      zone->publish_motion(false);
    }
    return true;
  }
  ESP_LOGW(TAG, "set_zone: unknown zone id %d", zone_id);
  return false;
}

bool FP2Component::reset_zone_config(int zone_id) {
  for (auto *zone : zones_) {
    if (zone->id != zone_id) continue;
    zone->grid = zone->yaml_grid;
    zone->sensitivity = zone->yaml_sensitivity;
    zone->runtime_override = false;
    save_zone_overrides_();
    ESP_LOGI(TAG, "Zone %u reset to YAML config", zone->id);
    if (init_done_) {
      send_zone_to_radar_(zone);
      send_zone_activation_list_();
    }
    publish_zone_map_(zone);
    return true;
  }
  return false;
}

void FP2Component::dump_config() {
  ESP_LOGCONFIG(TAG, "Aqara FP2:");
  ESP_LOGCONFIG(TAG, "  Mounting Position: %d", mounting_position_);
  ESP_LOGCONFIG(TAG, "  Zones: %d", zones_.size());
  ESP_LOGCONFIG(TAG, "  Proximity: %d  Direction: %d  AI person detection: %s  People counting: %s",
                proximity_, detection_direction_, YESNO(ai_person_detection_), YESNO(people_counting_));
  ESP_LOGCONFIG(TAG, "  Fall detection: %s  Sleep monitoring: %s", YESNO(fall_detection_), YESNO(sleep_enabled_));
  if (reset_pin_ != nullptr) {
    LOG_PIN("  Reset Pin: ", reset_pin_);
  }
  for (auto &z : zones_) {
    if (z->presence_sensor != nullptr) {
      LOG_BINARY_SENSOR("  ", "Zone Presence", z->presence_sensor);
    }
    if (z->motion_sensor != nullptr) {
      LOG_BINARY_SENSOR("  ", "Zone Motion", z->motion_sensor);
    }
  }
}

JsonDocument FP2Component::get_map_config_json() {
  JsonDocument doc;

  // Deserialize the compile-time JSON
  DeserializationError error = deserializeJson(doc, this->map_config_json_);

  if (error) {
    ESP_LOGE(TAG, "Failed to parse map config JSON: %s", error.c_str());
    return doc;
  }

  // The base structure is already in the JSON, but we can add runtime data if needed
  // For example, current zone states could be added here in the future

  return doc;
}

const char* FP2Component::get_mounting_position_string_() {
  switch (mounting_position_) {
    case 0x02: return "left_upper_corner";
    case 0x03: return "right_upper_corner";
    default: return "wall";
  }
}

// ESPHome >= 2025.x no longer has get_object_id(); build the id into a buffer.
static std::string entity_object_id(EntityBase *e) {
  char buf[OBJECT_ID_MAX_LEN];
  StringRef ref = e->get_object_id_to(std::span<char, OBJECT_ID_MAX_LEN>(buf, OBJECT_ID_MAX_LEN));
  return std::string(ref.c_str(), ref.size());
}

void FP2Component::json_get_map_data(JsonObject root) {
  // Global settings
  root["mounting_position"] = get_mounting_position_string_();
  root["left_right_reverse"] = left_right_reverse_;
  root["global_sensitivity"] = global_presence_sensitivity_;
  root["editable"] = true;
  root["grid_rows"] = 20;
  root["grid_cols"] = 16;
  root["grid_format"] = "rows20";
  root["cell_size_m"] = 0.5;
  {
    // active view for the current mounting mode: [col0, row0, cols, rows]
    JsonArray view = root["view"].to<JsonArray>();
    if (mounting_position_ == 0x02 || mounting_position_ == 0x03) {
      view.add(2); view.add(0); view.add(14); view.add(14);
    } else {
      view.add(0); view.add(0); view.add(16); view.add(20);
    }
  }
  root["sleep_enabled"] = sleep_enabled_;
  if (presence_event_sensor_ != nullptr) root["presence_event_object_id"] = entity_object_id(presence_event_sensor_);
  if (people_count_sensor_ != nullptr) root["people_count_object_id"] = entity_object_id(people_count_sensor_);
  if (sleep_presence_sensor_ != nullptr) root["sleep_presence_object_id"] = entity_object_id(sleep_presence_sensor_);
  if (sleep_state_sensor_ != nullptr) root["sleep_state_object_id"] = entity_object_id(sleep_state_sensor_);
  if (radar_temperature_sensor_ != nullptr) root["radar_temperature_object_id"] = entity_object_id(radar_temperature_sensor_);
  if (target_tracking_sensor_ != nullptr) root["targets_object_id"] = entity_object_id(target_tracking_sensor_);

  // Global maps (always present; all-zero when not configured)
  root["interference_grid"] = grid_to_hex_card_format(interference_grid_);
  root["exit_grid"] = grid_to_hex_card_format(exit_grid_);
  root["edge_grid"] = grid_to_hex_card_format(edge_grid_);
  root["interference_override"] = map_override_[0];
  root["edge_override"] = map_override_[1];
  root["exit_override"] = map_override_[2];

  // Zones
  if (!zones_.empty()) {
    JsonArray zones_array = root["zones"].to<JsonArray>();
    for (FP2Zone *zone : zones_) {
      JsonObject zone_obj = zones_array.add<JsonObject>();
      zone_obj["id"] = zone->id;
      zone_obj["sensitivity"] = zone->sensitivity;
      zone_obj["runtime_override"] = zone->runtime_override;
      zone_obj["empty"] = zone->is_empty();
      zone_obj["grid"] = grid_to_hex_card_format(zone->grid);
      if (zone->presence_sensor != nullptr) {
        zone_obj["name"] = zone->presence_sensor->get_name().c_str();
        zone_obj["presence_sensor"] = zone->presence_sensor->get_name().c_str();  // legacy
        zone_obj["presence_object_id"] = entity_object_id(zone->presence_sensor);
      }
      if (zone->motion_sensor != nullptr) {
        zone_obj["motion_object_id"] = entity_object_id(zone->motion_sensor);
      }
      if (zone->event_sensor != nullptr) {
        zone_obj["event_object_id"] = entity_object_id(zone->event_sensor);
      }
    }
  }
}

} // namespace aqara_fp2
} // namespace esphome
