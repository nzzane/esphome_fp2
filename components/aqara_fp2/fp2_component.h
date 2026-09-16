#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/select/select.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/preferences.h"

#include "../aqara_fp2_accel/aqara_fp2_accel.h"

#include <ArduinoJson.h>
#include <array>
#include <cstdint>
#include <deque>
#include <vector>

namespace esphome {
namespace aqara_fp2 {

static const char *const TAG = "aqara_fp2";

// 40-byte Grid Map
using GridMap = std::array<uint8_t, 40>;

struct FP2Zone : public Component {
  FP2Zone(uint8_t zone_id, const GridMap grid, uint8_t sensitivity)
      : id(zone_id), grid(grid), sensitivity(sensitivity), yaml_grid(grid), yaml_sensitivity(sensitivity) {}

  bool is_empty() const {
    for (auto b : grid) if (b) return false;
    return true;
  }

  void set_presence_sensor(binary_sensor::BinarySensor *sensor) {
    this->presence_sensor = sensor;
  }

  void set_motion_sensor(binary_sensor::BinarySensor *sensor) {
    this->motion_sensor = sensor;
  }

  void set_map_sensor(text_sensor::TextSensor *sensor) {
    this->map_sensor = sensor;
  }

  void set_event_sensor(text_sensor::TextSensor *sensor) {
    this->event_sensor = sensor;
  }

  void publish_event(const std::string &ev) {
    if (this->event_sensor != nullptr) {
      this->event_sensor->publish_state(ev);
    }
  }

  void publish_presence(bool state) {
    if (this->presence_sensor != nullptr) {
      this->presence_sensor->publish_state(state);
    }
  }

  void publish_motion(bool state) {
    if (this->motion_sensor != nullptr) {
      this->motion_sensor->publish_state(state);
    }
  }

  void publish_map(const std::string &map_hex) {
    if (this->map_sensor != nullptr) {
      this->map_sensor->publish_state(map_hex);
    }
  }

  uint8_t id;
  esphome::binary_sensor::BinarySensor *presence_sensor{nullptr};
  esphome::binary_sensor::BinarySensor *motion_sensor{nullptr};
  esphome::text_sensor::TextSensor *map_sensor{nullptr};
  esphome::text_sensor::TextSensor *event_sensor{nullptr};
  GridMap grid;
  uint8_t sensitivity; // 1=Low, 2=Med, 3=High
  GridMap yaml_grid;
  uint8_t yaml_sensitivity;
  bool runtime_override{false};
};

// Runtime zone overrides persisted in flash (one slot per YAML zone, by index)
static const uint8_t MAX_RUNTIME_ZONES = 12;
struct ZoneOverride {
  uint8_t valid;
  uint8_t sensitivity;
  uint8_t grid[40];
} __attribute__((packed));
struct ZoneStore {
  uint32_t magic;
  ZoneOverride zones[MAX_RUNTIME_ZONES];
} __attribute__((packed));
static const uint32_t ZONE_STORE_MAGIC = 0x46503201;  // "FP2" v1

// Runtime overrides for the three global maps (interference / edge / entry-exit)
struct MapStore {
  uint32_t magic;
  uint8_t valid[3];
  uint8_t maps[3][40];
} __attribute__((packed));
static const uint32_t MAP_STORE_MAGIC = 0x4650324d;  // "FP2M"

class FP2Component;

enum class DataType : uint8_t {
    UINT8 = 0x00,
    UINT16 = 0x01,
    UINT32 = 0x02,
    VOID = 0x03,
    BOOL = 0x04,
    STRING = 0x05,
    BINARY = 0x06,
};

enum class OpCode : uint8_t {
  // Device -> Host: Standard Response to Read (Values).
  // Device -> Host: Reverse Read Request (SubID only, len=2).
  RESPONSE = 0x01,

  // Host -> Device: Write Attribute (Values).
  WRITE = 0x02,

  // Both: Acknowledge.
  ACK = 0x03,

  // Host -> Device: Standard Read Request (SubID only).
  // Host -> Device: Reverse Read Response (Values, in response to 0x01 Query).
  READ = 0x04,

  // Device -> Host: Async Report.
  REPORT = 0x05,
};

enum class AttrId : uint16_t {
    RADAR_SW_VERSION                = 0x0102,
    WORK_MODE                       = 0x0116,
    MONITOR_MODE                    = 0x0105, // Detection direction (0=default, 1=L/R)
    LEFT_RIGHT_REVERSE              = 0x0122, // L/R swap (0/1/2)
    PRESENCE_DETECT_SENSITIVITY     = 0x0111, // Sensitivity (1-3)
    CLOSING_SETTING                 = 0x0106, // Proximity (0=far, 1=med, 2=close)
    ZONE_CLOSE_AWAY_ENABLE          = 0x0153, // Zone N close/away enable
    FALL_SENSITIVITY                = 0x0123, // Fall sensitivity
    PEOPLE_COUNT_REPORT_ENABLE      = 0x0158, // People counting enable
    PEOPLE_NUMBER_ENABLE            = 0x0162, // People number enable
    TARGET_TYPE_ENABLE              = 0x0163, // AI person detection
    SLEEP_MOUNT_POSITION            = 0x0168, // Sleep mount position
    SLEEP_ZONE_SIZE                 = 0x0169, // Sleep zone dimensions
    WALL_CORNER_POS                 = 0x0170, // Wall/corner position
    DWELL_TIME_ENABLE               = 0x0172, // Dwell tracking
    WALK_DISTANCE_ENABLE            = 0x0173, // Walking distance
    INTERFERENCE_MAP                = 0x0110, // Interference map (40B)
    RESET_ABSENT_STATUS             = 0x0113, // BOOL, write
    EDGE_AUTO_ENABLE                = 0x0150, // BOOL
    INTERFERENCE_AUTO_ENABLE        = 0x0139, // BOOL
    ENTRY_EXIT_MAP                  = 0x0109, // Enter/exit zones (40B)
    EDGE_MAP                        = 0x0107, // Detection boundary (40B)
    ZONE_MAP                        = 0x0114, // Zone N area map (1B ID + 40B)
    ZONE_SENSITIVITY                = 0x0151, // Zone N sensitivity
    ZONE_ACTIVATION_LIST            = 0x0202, // Auxiliary config (32B)
    DETECT_ZONE_TYPE                = 0x0152, // Zone N type
    DEVICE_DIRECTION                = 0x0143,
    ANGLE_SENSOR_DATA               = 0x0120,
    LOCATION_REPORT_ENABLE          = 0x0112,
    ZONE_PRESENCE                   = 0x0142,
    LOCATION_TRACKING_DATA          = 0x0117,
    THERMO_EN                       = 0x0138,
    THERMO_DATA                     = 0x0141,
    TEMPERATURE                     = 0x0128,
    DETECT_ZONE_MOTION              = 0x0115,
    MOTION_DETECT                   = 0x0103,
    PRESENCE_DETECT                 = 0x0104,
    ONTIME_PEOPLE_NUMBER            = 0x0165,
    REALTIME_PEOPLE_NUMBER          = 0x0164,
    PEOPLE_COUNTING                 = 0x0155,
    FALL_DETECT_ENABLE              = 0x0121,
    POSTURE_REPORT_ENABLE           = 0x0157,
    TARGET_POSTURE                  = 0x0154,
    // Sleep monitoring (experimental - enum values not yet verified)
    SLEEP_REPORT_ENABLE             = 0x0156,
    SLEEP_DATA                      = 0x0159,
    SLEEP_STATE                     = 0x0161,
    SLEEP_PRESENCE                  = 0x0167,
    SLEEP_INOUT                     = 0x0171,
    SLEEP_EVENT                     = 0x0176,
    INVALID                         = 0xFFFF,
};

struct FP2Command {
  OpCode type;
  AttrId attr_id;
  std::vector<uint8_t> data;
  uint32_t last_send_time;
  uint8_t retry_count;
  // ACKs and reverse-read responses must echo the radar's sequence number;
  // -1 means "use our own tx counter".
  int16_t seq{-1};
};

// Runtime-changeable settings (mirrors the options in the Aqara app)
enum class Setting : uint8_t {
  MOUNTING_POSITION,     // select: wall / left_corner / right_corner
  PROXIMITY,             // select: far / medium / close
  DETECTION_DIRECTION,   // select: default / left_right
  SENSITIVITY,           // select: low / medium / high (global)
  FALL_SENSITIVITY,      // select: low / medium / high
  LEFT_RIGHT_REVERSE,    // switch
  AI_PERSON_DETECTION,   // switch
  PEOPLE_COUNTING,       // switch
  FALL_DETECTION,        // switch
  SLEEP_MONITORING,      // switch
};

struct RuntimeSettings {
  uint32_t magic;
  uint8_t mounting_position;
  uint8_t proximity;
  uint8_t detection_direction;
  uint8_t sensitivity;
  uint8_t fall_sensitivity;
  uint8_t left_right_reverse;
  uint8_t ai_person_detection;
  uint8_t people_counting;
  uint8_t fall_detection;
  uint8_t sleep_monitoring;
} __attribute__((packed));
static const uint32_t SETTINGS_MAGIC = 0x46503254;  // "FP2T" (v2: mounting default changed)

class FP2SettingSelect : public select::Select, public Component {
 public:
  void set_parent(FP2Component *parent, Setting setting) { parent_ = parent; setting_ = setting; }
  Setting get_setting() const { return setting_; }
 protected:
  void control(const std::string &value) override;
  FP2Component *parent_{nullptr};
  Setting setting_;
};

class FP2SettingSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(FP2Component *parent, Setting setting) { parent_ = parent; setting_ = setting; }
  Setting get_setting() const { return setting_; }
 protected:
  void write_state(bool state) override;
  FP2Component *parent_{nullptr};
  Setting setting_;
};

class FP2LocationSwitch : public switch_::Switch, public Component {
public:
  void set_parent(FP2Component *parent) { parent_ = parent; }
  void setup() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

protected:
  void write_state(bool state) override;
  FP2Component *parent_{nullptr};
};

class FP2Component : public Component, public uart::UARTDevice {
public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  // Configuration setters
  void set_radar_reset_pin(GPIOPin *pin) { reset_pin_ = pin; }
  void set_mounting_position(uint8_t pos) { mounting_position_ = pos; }
  void set_left_right_reverse(bool val) { left_right_reverse_ = val; }

  void set_fall_detection_sensitivity(uint8_t val) {
    fall_detection_sensitivity_ = val;
  }
  void set_fall_detection(bool val) { fall_detection_ = val; }
  void set_derive_presence(bool v) { derive_presence_ = v; }
  void set_absence_timeout(uint32_t ms) { absence_timeout_ms_ = ms; }
  // Debug: report a fixed orientation/angle to the radar instead of the accelerometer's
  void set_force_direction(int dir, int angle) { force_direction_ = dir; force_angle_ = angle; }
  // Debug: replay the stock firmware's boot writes verbatim instead of our init
  void set_replay_stock_init(bool v) { replay_stock_init_ = v; }
  void set_proximity(uint8_t val) { proximity_ = val; }
  void set_detection_direction(uint8_t val) { detection_direction_ = val; }
  void set_ai_person_detection(bool val) { ai_person_detection_ = val; }
  void set_people_counting(bool val) { people_counting_ = val; }
  void set_presence_event_sensor(text_sensor::TextSensor *sensor) { presence_event_sensor_ = sensor; }
  void set_people_count_sensor(sensor::Sensor *sensor) { people_count_sensor_ = sensor; }

  // Sleep monitoring (experimental)
  void set_sleep_enabled(bool val) { sleep_enabled_ = val; }
  void set_sleep_mount_position(uint8_t val) { sleep_mount_position_ = val; }
  void set_sleep_bed_size(uint16_t width, uint16_t length) {
    sleep_bed_width_ = width;
    sleep_bed_length_ = length;
  }
  void set_sleep_presence_sensor(binary_sensor::BinarySensor *sensor) { sleep_presence_sensor_ = sensor; }
  void set_sleep_state_sensor(sensor::Sensor *sensor) { sleep_state_sensor_ = sensor; }
  void set_sleep_inout_sensor(sensor::Sensor *sensor) { sleep_inout_sensor_ = sensor; }
  void set_sleep_event_sensor(sensor::Sensor *sensor) { sleep_event_sensor_ = sensor; }
  void set_sleep_data_sensor(text_sensor::TextSensor *sensor) { sleep_data_sensor_ = sensor; }

  void set_interference_grid(const std::vector<uint8_t> &grid);
  void set_exit_grid(const std::vector<uint8_t> &grid);
  void set_edge_grid(const std::vector<uint8_t> &grid);

  void set_presence_sensitivity(uint8_t val) { global_presence_sensitivity_ = val; }
  void set_motion_sensor(binary_sensor::BinarySensor *sensor) { global_motion_sensor_ = sensor; }
  void set_presence_sensor(binary_sensor::BinarySensor *sensor) { global_presence_sensor_ = sensor; }

  //void add_zone(uint8_t id, binary_sensor::BinarySensor *sens,
  //              const std::vector<uint8_t> &grid, uint8_t sensitivity);
  void set_zones(const std::vector<FP2Zone*> &zones);

  void set_target_tracking_sensor(text_sensor::TextSensor *sensor) {
    target_tracking_sensor_ = sensor;
  }
  void set_location_report_switch(FP2LocationSwitch *sw) {
    location_report_switch_ = sw;
    sw->set_parent(this);
  }

  void set_edge_label_grid_sensor(text_sensor::TextSensor *sensor) {
    ESP_LOGI(TAG, "set_edge_label_grid_sensor called (has_edge_grid_=%d)", has_edge_grid_);
    edge_label_grid_sensor_ = sensor;
    if (has_edge_grid_ && edge_label_grid_sensor_ != nullptr) {
      ESP_LOGI(TAG, "Publishing edge label grid from setter");
      edge_label_grid_sensor_->publish_state(grid_to_hex_card_format(edge_grid_));
    } else {
      ESP_LOGW(TAG, "NOT publishing edge label grid from setter (has_grid=%d, sensor=%p)", has_edge_grid_, edge_label_grid_sensor_);
    }
  }
  void set_entry_exit_grid_sensor(text_sensor::TextSensor *sensor) {
    ESP_LOGI(TAG, "set_entry_exit_grid_sensor called (has_exit_grid_=%d)", has_exit_grid_);
    entry_exit_grid_sensor_ = sensor;
    if (has_exit_grid_ && entry_exit_grid_sensor_ != nullptr) {
      ESP_LOGI(TAG, "Publishing entry/exit grid from setter");
      entry_exit_grid_sensor_->publish_state(grid_to_hex_card_format(exit_grid_));
    } else {
      ESP_LOGW(TAG, "NOT publishing entry/exit grid from setter (has_grid=%d, sensor=%p)", has_exit_grid_, entry_exit_grid_sensor_);
    }
  }
  void set_interference_grid_sensor(text_sensor::TextSensor *sensor) {
    ESP_LOGI(TAG, "set_interference_grid_sensor called (has_interference_grid_=%d)", has_interference_grid_);
    interference_grid_sensor_ = sensor;
    if (has_interference_grid_ && interference_grid_sensor_ != nullptr) {
      ESP_LOGI(TAG, "Publishing interference grid from setter");
      interference_grid_sensor_->publish_state(grid_to_hex_card_format(interference_grid_));
    } else {
      ESP_LOGW(TAG, "NOT publishing interference grid from setter (has_grid=%d, sensor=%p)", has_interference_grid_, interference_grid_sensor_);
    }
  }
  void set_mounting_position_sensor(text_sensor::TextSensor *sensor) {
    mounting_position_sensor_ = sensor;
    if (mounting_position_sensor_ != nullptr) {
      const char* pos_str;
      switch (mounting_position_) {
        case 0x02: pos_str = "left_upper_corner"; break;
        case 0x03: pos_str = "right_upper_corner"; break;
        default: pos_str = "wall"; break;
      }
      mounting_position_sensor_->publish_state(pos_str);
    }
  }
  void set_radar_temperature_sensor(sensor::Sensor *sensor) {
      radar_temperature_sensor_ = sensor;
  }
  void set_radar_software_sensor(text_sensor::TextSensor *sensor) {
      radar_software_sensor_ = sensor;
  }

  void set_fp2_accel(aqara_fp2_accel::AqaraFP2Accel *accel) {
      fp2_accel_ = accel;
  }

  void set_location_reporting_enabled(bool enabled);

  // Grid format conversion
  std::string grid_to_hex_card_format(const GridMap &grid);
  static bool hex_card_format_to_grid(const std::string &hex, GridMap &grid);

  // Runtime settings (HA selects / switches)
  void add_setting_select(FP2SettingSelect *sel) { setting_selects_.push_back(sel); sel->set_parent(this, sel->get_setting()); }
  void add_setting_switch(FP2SettingSwitch *sw) { setting_switches_.push_back(sw); sw->set_parent(this, sw->get_setting()); }
  void apply_setting(Setting setting, uint8_t value);
  uint8_t get_setting_value(Setting setting) const;

  // Reset + re-initialise the radar (button / api action)
  void reset_radar() { restart_initialization_("requested"); }

  // Runtime zone editing (from the HA card via api actions)
  bool set_zone_config(int zone_id, const std::string &grid_hex, int sensitivity);
  bool reset_zone_config(int zone_id);
  // kind: "interference" (0x0110), "edge" (0x0107, cells the radar ignores), "exit" (0x0109)
  bool set_map_config(const std::string &kind, const std::string &grid_hex);
  bool reset_map_config(const std::string &kind);

  // Map configuration
  void set_map_config_json(const std::string &json) { map_config_json_ = json; }
  JsonDocument get_map_config_json();
  void json_get_map_data(JsonObject root);

protected:
  // Internal logic
  void process_command_queue_();
  void send_next_command_();
  void handle_incoming_byte_(uint8_t byte);
  const char* get_mounting_position_string_();
  void handle_parsed_frame_(uint8_t type, AttrId attr_id,
                            const std::vector<uint8_t> &payload);
  void handle_ack_(AttrId attr_id);
  void handle_report_(AttrId attr_id, const std::vector<uint8_t> &payload);
  void handle_location_tracking_report_(const std::vector<uint8_t> &payload);
  void handle_temperature_report_(const std::vector<uint8_t> &payload);
  void handle_sleep_report_(AttrId attr_id, const std::vector<uint8_t> &payload);
  void check_heartbeat_watchdog_();
  void load_settings_();
  void save_settings_();
  void publish_settings_();
  std::vector<FP2SettingSelect *> setting_selects_;
  std::vector<FP2SettingSwitch *> setting_switches_;
  ESPPreferenceObject settings_pref_;
  void load_map_overrides_();
  void save_map_overrides_();
  void send_map_to_radar_(int kind);
  void publish_map_sensor_(int kind);
  int map_kind_(const std::string &kind) const;
  GridMap yaml_interference_grid_{}, yaml_exit_grid_{}, yaml_edge_grid_{};
  bool yaml_has_interference_{false}, yaml_has_exit_{false}, yaml_has_edge_{false};
  bool map_override_[3]{false, false, false};
  ESPPreferenceObject map_pref_;
  void load_zone_overrides_();
  void save_zone_overrides_();
  void send_zone_to_radar_(FP2Zone *zone);
  void send_zone_activation_list_();
  void publish_zone_map_(FP2Zone *zone);
  ESPPreferenceObject zone_pref_;
  void restart_initialization_(const char *reason);
  void handle_response_(AttrId attr_id, const std::vector<uint8_t> &payload);
  void handle_reverse_read_request_(AttrId attr_id);
  void send_ack_(AttrId attr_id, uint8_t seq);

  // Initialization
  void perform_reset_();
  void check_initialization_();

  aqara_fp2_accel::AqaraFP2Accel *fp2_accel_{nullptr};

  GPIOPin *reset_pin_{nullptr};
  bool init_done_{false};
  uint32_t last_heartbeat_millis_{0};

  // Configuration State
  uint8_t mounting_position_{0x01}; // Default Wall
  bool left_right_reverse_{false};
  uint8_t fall_detection_sensitivity_{1};
  bool fall_detection_{false};
  bool replay_stock_init_{false};
  int force_direction_{-1};
  int force_angle_{-1};
  uint8_t current_direction_() { return force_direction_ >= 0 ? (uint8_t) force_direction_ : (uint8_t) fp2_accel_->get_orientation(); }
  uint16_t current_angle_() { return force_angle_ >= 0 ? (uint16_t) force_angle_ : (uint16_t) fp2_accel_->get_output_angle_z(); }
  void send_orientation_reports_();
  uint8_t proximity_{1};            // 0=far 1=medium 2=close
  uint8_t detection_direction_{0};  // 0=default 1=left/right
  bool ai_person_detection_{true};
  bool people_counting_{true};
  text_sensor::TextSensor *presence_event_sensor_{nullptr};
  sensor::Sensor *people_count_sensor_{nullptr};

  // Sleep monitoring (experimental)
  bool sleep_enabled_{false};
  uint8_t sleep_mount_position_{1};
  uint16_t sleep_bed_width_{120};
  uint16_t sleep_bed_length_{180};
  binary_sensor::BinarySensor *sleep_presence_sensor_{nullptr};
  sensor::Sensor *sleep_state_sensor_{nullptr};
  sensor::Sensor *sleep_inout_sensor_{nullptr};
  sensor::Sensor *sleep_event_sensor_{nullptr};
  text_sensor::TextSensor *sleep_data_sensor_{nullptr};

  // Derived presence: radar FW 99 does not send 0x0103/0x0104/0x0142 events
  // (at least in wall mode), so presence/motion/zone occupancy are derived from
  // the target stream unless the radar reports them itself.
  bool derive_presence_{true};
  uint32_t absence_timeout_ms_{30000};
  int16_t motion_velocity_threshold_{15};
  uint32_t last_target_seen_ms_{0};
  bool derived_presence_state_{false};
  bool radar_presence_seen_{false};
  bool radar_zone_seen_{false};
  std::vector<uint32_t> zone_last_seen_ms_;
  void update_derived_states_(const std::vector<uint8_t> &payload, uint8_t count);
  void check_derived_absence_();
  bool zone_contains_(const FP2Zone *zone, int16_t x, int16_t y) const;

  // Radar watchdog: re-run the init sequence if the heartbeat stops
  static const uint32_t HEARTBEAT_TIMEOUT_MS = 15000;
  uint32_t init_time_millis_{0};

  // Grids (Optional)
  GridMap interference_grid_{};
  bool has_interference_grid_{false};
  GridMap exit_grid_{};
  bool has_exit_grid_{false};
  GridMap edge_grid_{};
  bool has_edge_grid_{false};

  // Global zone
  uint8_t global_presence_sensitivity_{2}; // Default Medium
  binary_sensor::BinarySensor *global_presence_sensor_{nullptr};
  binary_sensor::BinarySensor *global_motion_sensor_{nullptr};

  // Zones
  std::vector<FP2Zone*> zones_;
  text_sensor::TextSensor *target_tracking_sensor_{nullptr};
  FP2LocationSwitch *location_report_switch_{nullptr};
  bool location_reporting_active_{false};

  // Grid text sensors
  text_sensor::TextSensor *edge_label_grid_sensor_{nullptr};
  text_sensor::TextSensor *entry_exit_grid_sensor_{nullptr};
  text_sensor::TextSensor *interference_grid_sensor_{nullptr};
  text_sensor::TextSensor *mounting_position_sensor_{nullptr};

  sensor::Sensor *radar_temperature_sensor_{nullptr};
  text_sensor::TextSensor *radar_software_sensor_{nullptr};

  // Map Configuration (compile-time generated)
  std::string map_config_json_;

  // Communication State
  std::deque<FP2Command> command_queue_;

  // Frame Decoder State
  enum DecoderState {
    SYNC,
    VER_H,
    VER_L,
    SEQ,
    OPCODE,
    LEN_H,
    LEN_L,
    H_CHECK,
    PAYLOAD,
    CRC_L,
    CRC_H
  } state_{SYNC};

  uint8_t rx_seq_;
  uint8_t rx_opcode_;
  uint16_t rx_len_;
  std::vector<uint8_t> rx_payload_;
  uint16_t rx_crc_;

  // Rolling checksum for header
  uint16_t header_sum_{0};

  // RX diagnostics
  uint32_t rx_bytes_{0}, rx_frames_{0}, rx_dropped_sync_{0}, rx_ver_mismatch_{0}, rx_hdr_fail_{0}, rx_crc_fail_{0};
  uint32_t rx_stats_last_ms_{0};

  // Ack Manager
  // We track the SubID of tAttrId::INVALID command we are currently waiting for an ACK for.
  // 0xFFFF = Not waiting.
  AttrId waiting_for_ack_attr_id_{AttrId::INVALID};
  FP2Command in_flight_{};  // the WRITE currently awaiting its ACK
  void transmit_(FP2Command &cmd);
  uint32_t last_command_sent_millis_{0};
  static const uint32_t ACK_TIMEOUT_MS = 500;
  static const uint8_t MAX_RETRIES = 3;

  void enqueue_command_(OpCode type, AttrId attr_id, uint8_t byte_val);
  void enqueue_command_(OpCode type, AttrId attr_id, uint16_t word_val);
  void enqueue_command_(OpCode type, AttrId attr_id, bool bool_val);
  void enqueue_command_blob2_(AttrId attr_id,
                              const std::vector<uint8_t> &blob_content);
  void enqueue_read_(AttrId attr_id);
  void send_reverse_response_(AttrId attr_id, uint8_t byte_val, uint8_t seq);
  void send_reverse_response_(AttrId attr_id, uint16_t word_val, uint8_t seq);
};

} // namespace aqara_fp2
} // namespace esphome
