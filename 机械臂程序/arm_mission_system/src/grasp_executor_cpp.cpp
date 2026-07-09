#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace
{

std::string trim(const std::string & text)
{
  const auto first = std::find_if_not(text.begin(), text.end(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  });
  if (first == text.end()) {
    return "";
  }

  const auto last = std::find_if_not(text.rbegin(), text.rend(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  }).base();
  return std::string(first, last);
}

std::string to_lower_copy(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return text;
}

bool starts_with(const std::string & text, const std::string & prefix)
{
  return text.rfind(prefix, 0) == 0;
}

}  // namespace

class GraspExecutorCpp : public rclcpp::Node
{
public:
  explicit GraspExecutorCpp(const rclcpp::NodeOptions & options)
  : Node("grasp_executor", options)
  {
    default_label_ = get_or_declare_string("default_label", "plastic");
    joint1_ = get_or_declare_int("joint1", -90);
    joint2_ = get_or_declare_int("joint2", 0);
    joint3_ = get_or_declare_int("joint3", 0);
    joint4_ = get_or_declare_int("joint4", -83);
    joint5_ = get_or_declare_int("joint5", -6);
    joint6_ = get_or_declare_int("joint6", -1);
    drop_mode_ = to_lower_copy(get_or_declare_string("drop_mode", "single_bag"));
    drop_label_ = get_or_declare_string("drop_label", "Old_school_bag");
    auto_pick_from_detector_ = get_or_declare_bool("auto_pick_from_detector", true);
    min_pick_interval_sec_ = get_or_declare_double("min_pick_interval_sec", 8.0);
    min_target_shift_m_ = get_or_declare_double("min_target_shift_m", 0.0);
    hw_result_timeout_sec_ = get_or_declare_double("hw_result_timeout_sec", 45.0);

    default_profile_.approach_z =
      get_or_declare_int("default_profile.approach_z", get_or_declare_int("default_approach_z", 170));
    default_profile_.grasp_z =
      get_or_declare_int("default_profile.grasp_z", get_or_declare_int("default_grasp_z", 115));

    load_profile_parameters();
    current_label_ = default_label_;

    arm_busy_pub_ = create_publisher<std_msgs::msg::Bool>("/mission/arm_busy", 10);
    result_pub_ = create_publisher<std_msgs::msg::String>("/arm/grasp_result", 10);
    detection_enable_pub_ = create_publisher<std_msgs::msg::Bool>("/arm/detection_enable", 10);
    hw_request_pub_ = create_publisher<std_msgs::msg::String>("/arm/grasp_hw_request", 10);

    label_sub_ = create_subscription<std_msgs::msg::String>(
      "/mission/trash_label", 10,
      std::bind(&GraspExecutorCpp::on_label, this, std::placeholders::_1));
    pick_target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mission/arm_pick_target", 10,
      std::bind(&GraspExecutorCpp::on_pick_target, this, std::placeholders::_1));
    detector_target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mission/trash_pose", 10,
      std::bind(&GraspExecutorCpp::on_detector_target, this, std::placeholders::_1));
    hw_result_sub_ = create_subscription<std_msgs::msg::String>(
      "/arm/grasp_hw_result", 10,
      std::bind(&GraspExecutorCpp::on_hw_result, this, std::placeholders::_1));

    timeout_timer_ = create_wall_timer(500ms, std::bind(&GraspExecutorCpp::check_timeout, this));

    publish_detection_enable(true);
    RCLCPP_INFO(
      get_logger(),
      "C++ grasp executor started. profiles=%zu aliases=%zu drop_mode=%s drop_label=%s auto_pick=%s joints=[%d,%d,%d,%d,%d,%d]",
      grasp_profiles_.size(),
      label_aliases_.size(),
      drop_mode_.c_str(),
      drop_label_.c_str(),
      auto_pick_from_detector_ ? "true" : "false",
      joint1_,
      joint2_,
      joint3_,
      joint4_,
      joint5_,
      joint6_);
  }

private:
  struct HeightProfile
  {
    int approach_z {170};
    int grasp_z {115};
  };

  std::string get_or_declare_string(const std::string & name, const std::string & default_value)
  {
    if (!has_parameter(name)) {
      declare_parameter<std::string>(name, default_value);
    }
    return get_parameter(name).as_string();
  }

  int get_or_declare_int(const std::string & name, int default_value)
  {
    if (!has_parameter(name)) {
      declare_parameter<int64_t>(name, default_value);
    }
    return static_cast<int>(get_parameter(name).as_int());
  }

  double get_or_declare_double(const std::string & name, double default_value)
  {
    if (!has_parameter(name)) {
      declare_parameter<double>(name, default_value);
    }
    return get_parameter(name).as_double();
  }

  bool get_or_declare_bool(const std::string & name, bool default_value)
  {
    if (!has_parameter(name)) {
      declare_parameter<bool>(name, default_value);
    }
    return get_parameter(name).as_bool();
  }

  void load_profile_parameters()
  {
    const auto listed = list_parameters({}, 10);

    for (const auto & name : listed.names) {
      if (starts_with(name, "label_aliases.")) {
        const std::string raw_key = name.substr(std::string("label_aliases.").size());
        const std::string key = to_lower_copy(trim(raw_key));
        const std::string value = trim(get_parameter(name).as_string());
        if (!key.empty() && !value.empty()) {
          label_aliases_[key] = value;
        }
        continue;
      }

      if (starts_with(name, "grasp_profiles.")) {
        const std::string rest = name.substr(std::string("grasp_profiles.").size());
        const auto dot_pos = rest.find('.');
        if (dot_pos == std::string::npos) {
          continue;
        }

        const std::string label = trim(rest.substr(0, dot_pos));
        const std::string field = trim(rest.substr(dot_pos + 1));
        if (label.empty() || field.empty()) {
          continue;
        }

        auto iter = grasp_profiles_.find(label);
        if (iter == grasp_profiles_.end()) {
          iter = grasp_profiles_.emplace(label, default_profile_).first;
        }

        if (field == "approach_z") {
          iter->second.approach_z = static_cast<int>(get_parameter(name).as_int());
        } else if (field == "grasp_z") {
          iter->second.grasp_z = static_cast<int>(get_parameter(name).as_int());
        }
      }
    }
  }

  std::string normalize_label(const std::string & raw_label) const
  {
    const std::string normalized = to_lower_copy(trim(raw_label));
    if (normalized.empty()) {
      return "unknown";
    }

    const auto alias_iter = label_aliases_.find(normalized);
    if (alias_iter != label_aliases_.end()) {
      return alias_iter->second;
    }
    return normalized;
  }

  std::string resolve_drop_label(const std::string & source_label) const
  {
    if (drop_mode_ == "single_bag") {
      return drop_label_;
    }
    return source_label;
  }

  HeightProfile get_height_profile(const std::string & label) const
  {
    const auto normalized = normalize_label(label);
    const auto iter = grasp_profiles_.find(normalized);
    if (iter != grasp_profiles_.end()) {
      return iter->second;
    }
    return default_profile_;
  }

  void on_label(const std_msgs::msg::String::SharedPtr msg)
  {
    current_label_ = msg->data;
  }

  void on_pick_target(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(
      get_logger(),
      "Manual pick target received: x=%.3f y=%.3f label=%s",
      msg->pose.position.x,
      msg->pose.position.y,
      current_label_.c_str());
    start_pick(*msg, false);
  }

  void on_detector_target(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!auto_pick_from_detector_ || pick_in_progress_) {
      return;
    }

    const double now_sec = now().seconds();
    if ((now_sec - last_pick_time_sec_) < min_pick_interval_sec_) {
      return;
    }

    const double x = msg->pose.position.x;
    const double y = msg->pose.position.y;
    if (min_target_shift_m_ > 0.0 && has_last_pick_target_) {
      const double dx = x - last_pick_target_x_;
      const double dy = y - last_pick_target_y_;
      if (std::sqrt(dx * dx + dy * dy) < min_target_shift_m_) {
        return;
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Detector auto pick trigger: x=%.3f y=%.3f label=%s",
      x,
      y,
      current_label_.c_str());
    start_pick(*msg, true);
  }

  void start_pick(const geometry_msgs::msg::PoseStamped & target, bool from_detector)
  {
    if (pick_in_progress_) {
      return;
    }

    const std::string source_label = normalize_label(current_label_);
    const std::string execute_label = resolve_drop_label(source_label);
    const HeightProfile profile = get_height_profile(source_label);

    pick_in_progress_ = true;
    pick_started_sec_ = now().seconds();
    last_pick_time_sec_ = pick_started_sec_;
    last_pick_target_x_ = target.pose.position.x;
    last_pick_target_y_ = target.pose.position.y;
    has_last_pick_target_ = true;
    active_request_from_detector_ = from_detector;

    publish_busy(true);
    publish_detection_enable(false);

    std_msgs::msg::String request;
    request.data = build_request_payload(target, source_label, execute_label, profile);
    hw_request_pub_->publish(request);

    RCLCPP_INFO(
      get_logger(),
      "Grasp request sent: source_label=%s execute_label=%s approach_z=%d grasp_z=%d joints=[%d,%d,%d,%d,%d,%d]",
      source_label.c_str(),
      execute_label.c_str(),
      profile.approach_z,
      profile.grasp_z,
      joint1_,
      joint2_,
      joint3_,
      joint4_,
      joint5_,
      joint6_);
  }

  std::string build_request_payload(
    const geometry_msgs::msg::PoseStamped & target,
    const std::string & source_label,
    const std::string & execute_label,
    const HeightProfile & profile) const
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6);
    stream << "x=" << target.pose.position.x
           << ";y=" << target.pose.position.y
           << ";z=" << target.pose.position.z
           << ";frame_id=" << target.header.frame_id
           << ";source_label=" << source_label
           << ";execute_label=" << execute_label
           << ";approach_z=" << profile.approach_z
           << ";grasp_z=" << profile.grasp_z
           << ";joint1=" << joint1_
           << ";joint2=" << joint2_
           << ";joint3=" << joint3_
           << ";joint4=" << joint4_
           << ";joint5=" << joint5_
           << ";joint6=" << joint6_;
    return stream.str();
  }

  void on_hw_result(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!pick_in_progress_) {
      return;
    }
    finish_pick(msg->data);
  }

  void check_timeout()
  {
    if (!pick_in_progress_) {
      return;
    }

    const double elapsed = now().seconds() - pick_started_sec_;
    if (elapsed <= hw_result_timeout_sec_) {
      return;
    }

    RCLCPP_WARN(get_logger(), "Hardware grasp timeout after %.1f seconds", elapsed);
    finish_pick("grasp_failed:timeout");
  }

  void finish_pick(const std::string & result_text)
  {
    std_msgs::msg::String result_msg;
    result_msg.data = result_text;
    result_pub_->publish(result_msg);

    publish_busy(false);
    publish_detection_enable(true);
    pick_in_progress_ = false;

    if (result_text == "grasp_finished") {
      RCLCPP_INFO(
        get_logger(),
        "Grasp finished. source=%s",
        active_request_from_detector_ ? "detector" : "manual");
    } else {
      RCLCPP_WARN(get_logger(), "Grasp failed: %s", result_text.c_str());
    }
  }

  void publish_busy(bool busy)
  {
    std_msgs::msg::Bool msg;
    msg.data = busy;
    arm_busy_pub_->publish(msg);
  }

  void publish_detection_enable(bool enabled)
  {
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    detection_enable_pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_busy_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detection_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hw_request_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr label_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pick_target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr detector_target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr hw_result_sub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  std::unordered_map<std::string, std::string> label_aliases_;
  std::unordered_map<std::string, HeightProfile> grasp_profiles_;
  HeightProfile default_profile_;

  std::string default_label_;
  std::string current_label_;
  std::string drop_mode_;
  std::string drop_label_;

  int joint1_ {-90};
  int joint2_ {0};
  int joint3_ {0};
  int joint4_ {-83};
  int joint5_ {-6};
  int joint6_ {-1};

  bool auto_pick_from_detector_ {true};
  double min_pick_interval_sec_ {8.0};
  double min_target_shift_m_ {0.0};
  double hw_result_timeout_sec_ {45.0};

  bool pick_in_progress_ {false};
  bool has_last_pick_target_ {false};
  bool active_request_from_detector_ {false};
  double last_pick_time_sec_ {0.0};
  double pick_started_sec_ {0.0};
  double last_pick_target_x_ {0.0};
  double last_pick_target_y_ {0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<GraspExecutorCpp>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
