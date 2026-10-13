//
// mavros
// Copyright 2021 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

#include <gtest/gtest.h>

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <future>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "rcutils/logging.h"
#include "test_plugin_helpers.hpp"
#include "px4_events_test_helpers.hpp"
#include "../src/plugins/sys_status.cpp"  // NOLINT(build/include)

namespace mavros
{
namespace uas
{
namespace
{
std::mutex log_mutex;
std::vector<std::pair<int, std::string>> event_logs;

void capture_log(
  const rcutils_log_location_t *, int severity, const char *, rcutils_time_point_value_t,
  const char * format, va_list * args)
{
  char text[2048];
  va_list copy;
  va_copy(copy, *args);
  std::vsnprintf(text, sizeof(text), format, copy);
  va_end(copy);
  const std::string message(text);
  if (message.find("FCU: ") == 0 || message.find("PX4 event decoding inactive:") == 0 ||
    message.find("PX4 event decoding failed for ID ") == 0 ||
    message.find("PX4 event metadata ") == 0)
  {
    std::lock_guard<std::mutex> lock(log_mutex);
    event_logs.emplace_back(severity, message);
  }
}

class StatusEventTest : public TestUAS
{
protected:
  UAS::SharedPtr uas;
  std::shared_ptr<std_plugins::SystemStatusPlugin> plugin;
  rcutils_logging_output_handler_t previous_handler;
  int previous_level;

  void SetUp() override
  {
    previous_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(capture_log);
    std::lock_guard<std::mutex> lock(log_mutex);
    event_logs.clear();
  }

  void start(const std::string & name, bool px4 = true)
  {
    uas = create_uas(name);
    uas->set_tgt(1, 1);
    uas->update_heartbeat(
      utils::enum_value(mavlink::minimal::MAV_TYPE::QUADROTOR),
      utils::enum_value(px4 ? mavlink::minimal::MAV_AUTOPILOT::PX4 :
      mavlink::minimal::MAV_AUTOPILOT::ARDUPILOTMEGA), 0);
    plugin = std::make_shared<std_plugins::SystemStatusPlugin>(uas.get());
    register_plugin(uas, plugin);
    const auto logger = plugin->get_logger().get_name();
    previous_level = rcutils_logging_get_logger_level(logger);
    ASSERT_EQ(rcutils_logging_set_logger_level(logger, RCUTILS_LOG_SEVERITY_DEBUG), RCUTILS_RET_OK);
  }

  void TearDown() override
  {
    stop_spin();
    if (plugin) {
      EXPECT_EQ(
        rcutils_logging_set_logger_level(plugin->get_logger().get_name(), previous_level),
        RCUTILS_RET_OK);
    }
    rcutils_logging_set_output_handler(previous_handler);
  }

  void inject(uint32_t id = 0x01000007, uint8_t levels = 4)
  {
    mavlink::common::msg::EVENT event{};
    event.id = id;
    event.log_levels = levels;
    event.sequence = 123;
    event.event_time_boot_ms = 456;
    event.arguments[0] = 42;
    auto message = convert_message(event);
    message.sysid = 1;
    message.compid = 1;
    route(uas, &message, mavconn::Framing::ok);
  }

  std::vector<std::pair<int, std::string>> logs()
  {
    std::lock_guard<std::mutex> lock(log_mutex);
    return event_logs;
  }

  void expect_no_metadata_requests(const std::string & name, bool px4 = true)
  {
    auto server = std::make_shared<rclcpp::Node>(name + "_command_stub");
    auto requests = std::make_shared<std::atomic<unsigned>>(0);
    auto service = server->create_service<mavros_msgs::srv::CommandLong>(
      "/" + name + "/cmd/command",
      [requests](mavros_msgs::srv::CommandLong::Request::SharedPtr request,
      mavros_msgs::srv::CommandLong::Response::SharedPtr response) {
        if (request->param1 == mavlink::common::msg::COMPONENT_METADATA::MSG_ID ||
        request->param1 == mavlink::common::msg::COMPONENT_INFORMATION::MSG_ID)
        {
          ++*requests;
        }
        response->success = true;
      });
    start(name, px4);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(server);
    executor.add_node(plugin->get_node());
    uas->update_connection_status(true);
    const auto deadline = std::chrono::steady_clock::now() + 600ms;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    EXPECT_EQ(requests->load(), 0U);
    uas->update_connection_status(false);
  }
};

TEST_F(StatusEventTest, local_mode_logs_one_readable_line_and_preserves_raw_topic)
{
  start("test_event_local");
  auto client = make_client_node("status_event_client");
  std::promise<mavros_msgs::msg::StatusEvent> received;
  auto future = received.get_future();
  auto subscription = client->create_subscription<mavros_msgs::msg::StatusEvent>(
    "/test_event_local/status_event", rclcpp::SensorDataQoS(),
    [&received](mavros_msgs::msg::StatusEvent::ConstSharedPtr message) {
      received.set_value(*message);
    });
  add_node(plugin->get_node());
  start_spin();
  for (int i = 0; i < 100 && subscription->get_publisher_count() == 0; ++i) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GT(subscription->get_publisher_count(), 0u);
  inject();
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  const auto raw = future.get();
  EXPECT_EQ(raw.px4_id, 7u);
  EXPECT_EQ(raw.severity, 4);
  EXPECT_EQ(raw.sequence, 123);
  EXPECT_EQ(raw.arguments[0], 42);
  for (size_t i = 1; i < raw.arguments.size(); ++i) {
    EXPECT_EQ(raw.arguments[i], 0);
  }
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0].first, RCUTILS_LOG_SEVERITY_WARN);
  EXPECT_EQ(lines[0].second, "FCU: Warning next line");
}

TEST_F(StatusEventTest, ordinary_severities_follow_existing_logger_mapping)
{
  start("test_event_local");
  const std::array<int, 8> expected = {
    RCUTILS_LOG_SEVERITY_ERROR, RCUTILS_LOG_SEVERITY_ERROR,
    RCUTILS_LOG_SEVERITY_ERROR, RCUTILS_LOG_SEVERITY_ERROR,
    RCUTILS_LOG_SEVERITY_WARN, RCUTILS_LOG_SEVERITY_WARN,
    RCUTILS_LOG_SEVERITY_INFO, RCUTILS_LOG_SEVERITY_DEBUG};
  for (uint8_t severity = 0; severity < expected.size(); ++severity) {
    inject(0x01000007, severity);
  }
  const auto lines = logs();
  ASSERT_EQ(lines.size(), expected.size());
  for (size_t i = 0; i < lines.size(); ++i) {
    EXPECT_EQ(lines[i].first, expected[i]);
    EXPECT_EQ(lines[i].second, "FCU: Warning next line");
  }
}

TEST_F(StatusEventTest, disabled_protocol_events_are_debug_with_argument_details)
{
  start("test_event_local");
  inject(0x01000003, 0x38);
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0].first, RCUTILS_LOG_SEVERITY_DEBUG);
  EXPECT_EQ(lines[0].second, "FCU: protocol_summary [version=42, checks=0]");
}

TEST_F(StatusEventTest, unknown_event_retains_raw_log)
{
  start("test_event_local");
  inject(0x010000fe);
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0].first, RCUTILS_LOG_SEVERITY_WARN);
  EXPECT_EQ(lines[0].second.find("FCU: EVENT 254 with args -42-0-"), 0u);
}

TEST_F(StatusEventTest, disabled_by_default_retains_raw_disabled_severity_behavior)
{
  start("test_event_disabled");
  EXPECT_EQ(plugin->get_node()->get_parameter("event_decoding_mode").as_string(), "disabled");
  EXPECT_TRUE(plugin->get_node()->get_parameter("event_metadata_file").as_string().empty());
  inject(0x01000003, 8);
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0].first, RCUTILS_LOG_SEVERITY_WARN);
  EXPECT_EQ(lines[0].second.find("FCU: UNK(8): EVENT 3 with args -42-0-"), 0u);
}

TEST_F(StatusEventTest, non_px4_vehicle_retains_raw_log)
{
  start("test_event_local", false);
  inject();
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0].second.find("FCU: EVENT 7 with args -42-0-"), 0u);
}

TEST_F(StatusEventTest, invalid_local_file_reports_once_and_does_not_stop_raw_events)
{
  start("test_event_invalid");
  inject();
  inject();
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_EQ(lines[0].first, RCUTILS_LOG_SEVERITY_ERROR);
  EXPECT_EQ(lines[0].second.find("PX4 event decoding inactive:"), 0u);
  EXPECT_EQ(lines[1].second.find("FCU: EVENT 7 with args -42-0-"), 0u);
  EXPECT_EQ(lines[2].second, lines[1].second);
}

TEST_F(StatusEventTest, decoding_parameters_require_restart)
{
  start("test_event_local");
  const auto node = plugin->get_node();
  EXPECT_TRUE(node->describe_parameter("event_decoding_mode").read_only);
  EXPECT_TRUE(node->describe_parameter("event_metadata_file").read_only);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("event_decoding_mode",
          "disabled")).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("event_metadata_file",
          "/other.json")).successful);
  inject();
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0].second, "FCU: Warning next line");
}

TEST_F(StatusEventTest, auto_mode_requests_metadata_and_keeps_events_raw)
{
  using Command = mavros_msgs::srv::CommandLong;
  auto server = make_client_node("event_command_stub");
  auto received = std::make_shared<std::promise<Command::Request>>();
  auto future = received->get_future();
  auto requests = std::make_shared<std::atomic<unsigned>>(0);
  auto service = server->create_service<Command>(
    "/test_event_auto/cmd/command",
    [received, requests](Command::Request::SharedPtr request,
    Command::Response::SharedPtr response) {
      if (request->param1 == mavlink::common::msg::COMPONENT_METADATA::MSG_ID) {
        if (requests->fetch_add(1) == 0) {
          received->set_value(*request);
        }
      }
      response->success = true;
      response->result = utils::enum_value(mavlink::common::MAV_RESULT::ACCEPTED);
    });
  start("test_event_auto");
  add_node(plugin->get_node());
  start_spin();
  uas->update_connection_status(true);
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  const auto request = future.get();
  EXPECT_FALSE(request.broadcast);
  EXPECT_EQ(request.command, utils::enum_value(mavlink::common::MAV_CMD::REQUEST_MESSAGE));
  EXPECT_EQ(request.param1, mavlink::common::msg::COMPONENT_METADATA::MSG_ID);
  EXPECT_EQ(request.param7, 1.0f);

  mavlink::common::msg::COMPONENT_METADATA advertised{};
  mavlink::set_string(advertised.uri, "mftp://etc/extras/component_general.json.xz");
  advertised.file_crc = 123;
  auto message = convert_message(advertised);
  message.sysid = 1;
  message.compid = 2;
  route(uas, &message, mavconn::Framing::ok);
  EXPECT_TRUE(logs().empty());
  message.compid = 1;
  route(uas, &message, mavconn::Framing::ok);
  inject();
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 2U);
  EXPECT_EQ(
    lines[0].second, "PX4 event metadata advertised: mftp://etc/extras/component_general.json.xz");
  EXPECT_EQ(lines[1].second.find("FCU: EVENT 7 with args -42-0-"), 0U);
  uas->update_connection_status(false);
  route(uas, &message, mavconn::Framing::ok);
  EXPECT_EQ(logs().size(), 2U);
  stop_spin();
  EXPECT_EQ(requests->load(), 1U);
}

TEST_F(StatusEventTest, disabled_mode_does_not_request_metadata)
{
  expect_no_metadata_requests("test_event_disabled");
}

TEST_F(StatusEventTest, auto_mode_accepts_legacy_advertisement_fields)
{
  start("test_event_auto");
  uas->update_connection_status(true);
  mavlink::common::msg::COMPONENT_INFORMATION advertised{};
  mavlink::set_string(advertised.general_metadata_uri,
          "mftp://etc/extras/component_general.json.xz");
  advertised.general_metadata_file_crc = 456;
  auto message = convert_message(advertised);
  message.sysid = 1;
  message.compid = 1;
  route(uas, &message, mavconn::Framing::ok);
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 1U);
  EXPECT_EQ(
    lines[0].second, "PX4 event metadata advertised: mftp://etc/extras/component_general.json.xz");
  uas->update_connection_status(false);
}

TEST_F(StatusEventTest, local_mode_does_not_request_metadata)
{
  expect_no_metadata_requests("test_event_local");
}

TEST_F(StatusEventTest, non_px4_auto_mode_does_not_request_metadata)
{
  expect_no_metadata_requests("test_event_auto", false);
}

TEST_F(StatusEventTest, invalid_auto_local_path_does_not_start_discovery)
{
  expect_no_metadata_requests("test_event_auto_invalid");
  inject();
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 2U);
  EXPECT_EQ(lines[0].second.find("PX4 event decoding inactive:"), 0U);
  EXPECT_EQ(lines[1].second.find("FCU: EVENT 7 with args -42-0-"), 0U);
}

}  // namespace
}  // namespace uas
}  // namespace mavros

int main(int argc, char ** argv)
{
  const std::string parameters =
    "/test_event_local/sys:\n"
    "  ros__parameters:\n"
    "    event_decoding_mode: local\n"
    "    event_metadata_file: '" TEST_EVENT_METADATA "'\n"
    "/test_event_invalid/sys:\n"
    "  ros__parameters:\n"
    "    event_decoding_mode: local\n"
    "    event_metadata_file: /file/that/does/not/exist\n"
    "/test_event_auto/sys:\n"
    "  ros__parameters:\n"
    "    event_decoding_mode: auto\n"
    "/test_event_auto_invalid/sys:\n"
    "  ros__parameters:\n"
    "    event_decoding_mode: auto\n"
    "    event_metadata_file: /file/that/does/not/exist\n";
  mavros::px4_events::TemporaryMetadata file(parameters, ".yaml");
  const char * ros_args[] = {"mavros-status-event-test", "--ros-args", "--params-file",
    file.path.c_str()};
  rclcpp::init(4, ros_args);
  ::testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
