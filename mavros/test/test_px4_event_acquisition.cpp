/*
 * Copyright 2026 Sean Hickey.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Test two-file acquisition through mock FTP services
 * @file test_px4_event_acquisition.cpp
 * @author Sean Hickey
 */

#include <gtest/gtest.h>
#include <lzma.h>

#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "../src/plugins/px4_events/acquisition.hpp"

namespace mavros
{
namespace px4_events
{

class AcquisitionTestPeer
{
public:
  static bool waiting_retry(Acquisition & acquisition)
  {
    std::lock_guard<std::mutex> lock(acquisition.mutex_);
    return acquisition.phase_ == Acquisition::Phase::Retry;
  }

  static bool waiting_busy(Acquisition & acquisition)
  {
    std::lock_guard<std::mutex> lock(acquisition.mutex_);
    return acquisition.busy_retried_ && !acquisition.pending_ &&
           acquisition.phase_ != Acquisition::Phase::Retry;
  }

  static void retry(Acquisition & acquisition)
  {
    Acquisition::Clock::time_point next;
    {
      std::lock_guard<std::mutex> lock(acquisition.mutex_);
      next = acquisition.next_request_;
    }
    acquisition.tick(next);
  }

  static void expire_download(Acquisition & acquisition)
  {
    acquisition.tick(Acquisition::Clock::now() + std::chrono::seconds(11));
  }
};

namespace
{
using namespace std::chrono_literals;  // NOLINT
using Open = mavros_msgs::srv::FileOpen;
using Read = mavros_msgs::srv::FileRead;
using Close = mavros_msgs::srv::FileClose;

class AcquisitionTest : public ::testing::Test
{
protected:
  enum class ReadBehavior { Normal, BusyOnce, BusyAlways, Empty, Excess, Timeout };
  ReadBehavior read_behavior{ReadBehavior::Normal};
  const std::string definitions =
    R"({"version":2,"components":{"0":{
    "namespace":"common","event_groups":{"default":{"events":{
    "1":{"name":"test","message":"Decoded event"}}}}}}})";
  std::string events = definitions;
  std::string general;
  std::string general_path = "general.json";
  std::string event_path = "events.json";
  std::string hold_read;
  std::string hold_close;
  std::string fail_close;
  int open_error{0};
  std::optional<uint32_t> general_size;
  size_t general_padding{0};
  std::vector<uint64_t> offsets;
  std::vector<std::string> opens;
  std::vector<std::string> closes;
  unsigned completions{0};
  std::shared_ptr<rmw_request_id_t> held_header;
  std::optional<Acquisition::Result> result;
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<Acquisition> acquisition;
  rclcpp::Service<Open>::SharedPtr open_service;
  rclcpp::Service<Read>::SharedPtr read_service;
  rclcpp::Service<Close>::SharedPtr close_service;
  rclcpp::executors::SingleThreadedExecutor executor;

  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("acquisition_test", "/acquisition");
    open_service = node->create_service<Open>(
      "ftp/open", [this](Open::Request::SharedPtr request, Open::Response::SharedPtr response) {
        opens.push_back(request->file_path);
        EXPECT_EQ(request->mode, Open::Request::MODE_READ);
        response->success = open_error == 0;
        response->r_errno = open_error;
        response->size = request->file_path == general_path ?
        general_size.value_or(general.size()) : events.size();
      });
    read_service = node->create_service<Read>(
      "ftp/read",
      [this](std::shared_ptr<rmw_request_id_t> header, Read::Request::SharedPtr request) {
        EXPECT_LE(request->size, 239U);
        if (request->file_path == general_path) {
          offsets.push_back(request->offset);
        }
        if (request->file_path == hold_read) {
          held_header = header;
          return;
        }
        const auto & file = request->file_path == general_path ? general : events;
        const auto chunk = file.substr(request->offset, request->size);
        Read::Response response;
        response.success = true;
        response.data.assign(chunk.begin(), chunk.end());
        if (request->file_path == general_path) {
          if (read_behavior == ReadBehavior::BusyAlways ||
          (read_behavior == ReadBehavior::BusyOnce && offsets.size() == 1))
          {
            response.success = false;
            response.r_errno = EBUSY;
          } else if (read_behavior == ReadBehavior::Timeout) {
            response.success = false;
            response.r_errno = ETIMEDOUT;
          } else if (read_behavior == ReadBehavior::Empty) {
            response.data.clear();
          } else if (read_behavior == ReadBehavior::Excess) {
            response.data.resize(request->size + 1);
          }
        }
        read_service->send_response(*header, response);
      });
    close_service = node->create_service<Close>(
      "ftp/close",
      [this](std::shared_ptr<rmw_request_id_t> header, Close::Request::SharedPtr request) {
        closes.push_back(request->file_path);
        if (request->file_path == hold_close) {
          held_header = header;
          return;
        }
        Close::Response response;
        response.success = request->file_path != fail_close;
        response.r_errno = response.success ? 0 : EIO;
        close_service->send_response(*header, response);
      });
    acquisition = Acquisition::create(node);
    executor.add_node(node);
  }

  void TearDown() override
  {
    acquisition->cancel();
    executor.remove_node(node);
  }

  std::string compress(const std::string & bytes)
  {
    std::string output(lzma_stream_buffer_bound(bytes.size()), '\0');
    size_t end = 0;
    EXPECT_EQ(lzma_easy_buffer_encode(6, LZMA_CHECK_CRC64, nullptr,
        reinterpret_cast<const uint8_t *>(bytes.data()), bytes.size(),
        reinterpret_cast<uint8_t *>(output.data()), &end, output.size()), LZMA_OK);
    output.resize(end);
    return output;
  }

  bool spin_until(const std::function<bool ()> & done)
  {
    const auto deadline = std::chrono::steady_clock::now() + 4s;
    while (!done() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(1ms);
    }
    return done();
  }

  void start(bool compressed = false, bool bad_general_crc = false)
  {
    general_path = compressed ? "general.json.xz" : "general.json";
    event_path = compressed ? "events.json.xz" : "events.json";
    if (compressed) {
      events = compress(events);
    }
    auto json =
      std::string("{\"version\":1,\"metadataTypes\":[{\"type\":4,\"uri\":\"mftp://") +
      event_path + "\",\"fileCrc\":" + std::to_string(metadata_crc32(events)) + "}]}";
    json.append(general_padding, ' ');
    general = compressed ? compress(json) : json;
    result.reset();
    ASSERT_TRUE(acquisition->start(
        metadata_file("mftp://" + general_path, metadata_crc32(json) ^ (bad_general_crc ? 1U : 0U)),
        [this](Acquisition::Result value) {
          ++completions;
          result = std::move(value);
        }));
  }
};

TEST_F(AcquisitionTest, validates_plain_and_xz_files_and_returns_working_decoder)
{
  for (const bool compressed : {false, true}) {
    events = definitions;
    start(compressed);
    EXPECT_FALSE(acquisition->start(metadata_file("mftp://other", 0), [](Acquisition::Result) {}));
    ASSERT_TRUE(spin_until([this]() {return result.has_value();}));
    ASSERT_TRUE(result->error.empty()) << result->error;
    ASSERT_NE(result->decoder, nullptr);
    const auto decoded = result->decoder->decode(1, 6, {});
    ASSERT_TRUE(decoded);
    EXPECT_EQ(decoded->text, "Decoded event");
    EXPECT_EQ(opens[opens.size() - 2], general_path);
    EXPECT_EQ(opens.back(), event_path);
    EXPECT_EQ(closes.back(), event_path);
  }
}

TEST_F(AcquisitionTest, checksum_failure_restarts_whole_sequence_with_three_attempt_limit)
{
  start();
  events += " ";  // The advertised event CRC no longer matches.
  for (unsigned attempt = 0; attempt < 2; ++attempt) {
    ASSERT_TRUE(spin_until([this]() {return AcquisitionTestPeer::waiting_retry(*acquisition);}));
    AcquisitionTestPeer::retry(*acquisition);
  }
  ASSERT_TRUE(spin_until([this]() {return result.has_value();}));
  EXPECT_EQ(opens.size(), 6U);
  EXPECT_EQ(result->decoder, nullptr);
  EXPECT_NE(result->error.find("checksum"), std::string::npos);
}

TEST_F(AcquisitionTest, general_checksum_failure_never_downloads_events)
{
  start(false, true);
  for (unsigned attempt = 0; attempt < 2; ++attempt) {
    ASSERT_TRUE(spin_until([this]() {return AcquisitionTestPeer::waiting_retry(*acquisition);}));
    AcquisitionTestPeer::retry(*acquisition);
  }
  ASSERT_TRUE(spin_until([this]() {return result.has_value();}));
  EXPECT_EQ(opens, (std::vector<std::string>{general_path, general_path, general_path}));
}

TEST_F(AcquisitionTest, malformed_or_unsupported_metadata_stops_without_retry)
{
  events = "not JSON";
  start();
  ASSERT_TRUE(spin_until([this]() {return result.has_value();}));
  EXPECT_EQ(opens.size(), 2U);
  EXPECT_FALSE(result->error.empty());
  general = R"({"version":1,"metadataTypes":[{"type":4,"uri":"https://host/events","fileCrc":0}]})";
  result.reset();
  opens.clear();
  ASSERT_TRUE(acquisition->start(metadata_file("mftp://general.json", metadata_crc32(general)),
    [this](Acquisition::Result value) {result = std::move(value);}));
  ASSERT_TRUE(spin_until([this]() {return result.has_value();}));
  EXPECT_EQ(opens.size(), 1U);
  EXPECT_FALSE(result->error.empty());
}

TEST_F(AcquisitionTest, unresolved_general_read_or_close_stops_before_event_download)
{
  for (const bool close : {false, true}) {
    hold_read = close ? "" : "general.json";
    hold_close = close ? "general.json" : "";
    held_header.reset();
    opens.clear();
    start();
    ASSERT_TRUE(spin_until([this]() {return held_header != nullptr;}));
    AcquisitionTestPeer::expire_download(*acquisition);
    ASSERT_TRUE(spin_until([this]() {return result.has_value();}));
    EXPECT_EQ(opens.size(), 1U);
    EXPECT_TRUE(result->transport_uncertain);
    EXPECT_EQ(result->decoder, nullptr);
    const auto before = completions;
    if (!close) {
      Read::Response late;
      late.success = true;
      late.data.assign(general.begin(), general.end());
      read_service->send_response(*held_header, late);
    } else {
      Close::Response late;
      late.success = true;
      close_service->send_response(*held_header, late);
    }
    executor.spin_some();
    EXPECT_EQ(completions, before);
    EXPECT_EQ(opens.size(), 1U);
  }
}

TEST_F(AcquisitionTest, confirmed_close_failures_preserve_valid_definitions)
{
  for (const auto & path : {general_path, event_path}) {
    fail_close = path;
    opens.clear();
    start();
    ASSERT_TRUE(spin_until([this]() {return result.has_value();}));
    EXPECT_EQ(opens.size(), 2U);
    EXPECT_NE(result->decoder, nullptr);
  }
}

TEST_F(AcquisitionTest, unresolved_final_close_keeps_valid_definitions_but_never_retries)
{
  for (const bool corrupt : {false, true}) {
    events = definitions;
    hold_close = "events.json";
    held_header.reset();
    opens.clear();
    start();
    if (corrupt) {
      events += " ";
    }
    ASSERT_TRUE(spin_until([this]() {return held_header != nullptr;}));
    AcquisitionTestPeer::expire_download(*acquisition);
    ASSERT_TRUE(spin_until([this]() {return result.has_value();}));
    EXPECT_EQ(opens.size(), 2U);
    EXPECT_TRUE(result->transport_uncertain);
    EXPECT_EQ(result->decoder != nullptr, !corrupt);
  }
}

TEST_F(AcquisitionTest, missing_file_stops_but_confirmed_io_failure_can_retry)
{
  open_error = ENOENT;
  start();
  ASSERT_TRUE(spin_until([this]() {return result.has_value();}));
  EXPECT_EQ(opens.size(), 1U);
  open_error = EIO;
  start();
  ASSERT_TRUE(spin_until([this]() {return AcquisitionTestPeer::waiting_retry(*acquisition);}));
  open_error = 0;
  AcquisitionTestPeer::retry(*acquisition);
  ASSERT_TRUE(spin_until([this]() {return result.has_value();}));
  EXPECT_NE(result->decoder, nullptr);
}

TEST_F(AcquisitionTest, cancel_discards_late_response_and_allows_a_new_start)
{
  hold_read = "general.json";
  start();
  ASSERT_TRUE(spin_until([this]() {return held_header != nullptr;}));
  acquisition->cancel();
  ASSERT_TRUE(result);
  EXPECT_EQ(completions, 1U);
  Read::Response late;
  late.success = true;
  late.data.assign(general.begin(), general.end());
  read_service->send_response(*held_header, late);
  hold_read.clear();
  start();
  ASSERT_TRUE(spin_until([this]() {return result.has_value();}));
  EXPECT_EQ(completions, 2U);
  EXPECT_NE(result->decoder, nullptr);
}

TEST_F(AcquisitionTest, chunks_in_order_and_completion_can_reenter_and_throw)
{
  general_padding = 500;
  start();
  ASSERT_TRUE(spin_until([this]() {return result.has_value();}));
  ASSERT_NE(result->decoder, nullptr);
  EXPECT_EQ(offsets, (std::vector<uint64_t>{0, 239, 478}));
  auto reentered = std::make_shared<bool>(false);
  ASSERT_TRUE(acquisition->start(metadata_file("mftp://general.json", metadata_crc32(general)),
    [this, reentered](Acquisition::Result value) {
      EXPECT_NE(value.decoder, nullptr);
      acquisition->cancel();
      *reentered = true;
      throw std::runtime_error("injected completion failure");
      }));
  ASSERT_TRUE(spin_until([reentered]() {return *reentered;}));
}

TEST_F(AcquisitionTest, busy_retries_same_offset_once_then_continues_without_pacing)
{
  general_padding = 500;
  read_behavior = ReadBehavior::BusyOnce;
  start();
  ASSERT_TRUE(spin_until([this]() {return AcquisitionTestPeer::waiting_busy(*acquisition);}));
  EXPECT_EQ(offsets.size(), 1U);
  AcquisitionTestPeer::retry(*acquisition);
  ASSERT_TRUE(spin_until([this]() {return result.has_value();}));
  EXPECT_NE(result->decoder, nullptr);
  EXPECT_EQ(offsets, (std::vector<uint64_t>{0, 0, 239, 478}));
}

TEST_F(AcquisitionTest, repeated_busy_bounds_operation_and_whole_attempt_retries)
{
  read_behavior = ReadBehavior::BusyAlways;
  start();
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    ASSERT_TRUE(spin_until([this]() {return AcquisitionTestPeer::waiting_busy(*acquisition);}));
    AcquisitionTestPeer::retry(*acquisition);
    if (attempt < 2) {
      ASSERT_TRUE(spin_until([this]() {return AcquisitionTestPeer::waiting_retry(*acquisition);}));
      EXPECT_EQ(offsets.size(), 2U * (attempt + 1));
      AcquisitionTestPeer::retry(*acquisition);
    }
  }
  ASSERT_TRUE(spin_until([this]() {return result.has_value();}));
  EXPECT_EQ(offsets.size(), 6U);
  EXPECT_EQ(closes.size(), 3U);
  EXPECT_EQ(result->decoder, nullptr);
}

TEST_F(AcquisitionTest, rejects_long_paths_size_limits_and_invalid_read_progress)
{
  ASSERT_TRUE(acquisition->start(metadata_file("mftp://" + std::string(239, 'a'), 0),
    [this](Acquisition::Result value) {result = std::move(value);}));
  ASSERT_TRUE(result);
  EXPECT_FALSE(result->error.empty());
  EXPECT_TRUE(opens.empty());
  for (const auto size : {0U, static_cast<uint32_t>(Decoder::MAX_COMPRESSED_BYTES + 1)}) {
    general_size = size;
    events = definitions;
    start(true);
    ASSERT_TRUE(spin_until([this]() {return result.has_value();}));
    EXPECT_FALSE(result->error.empty());
    EXPECT_TRUE(offsets.empty());
    EXPECT_EQ(result->decoder, nullptr);
  }
  EXPECT_EQ(closes.size(), 2U);
  general_size.reset();
  for (const auto behavior : {ReadBehavior::Empty, ReadBehavior::Excess}) {
    read_behavior = behavior;
    events = definitions;
    start();
    ASSERT_TRUE(spin_until([this]() {return result.has_value();}));
    EXPECT_FALSE(result->error.empty());
    EXPECT_EQ(result->decoder, nullptr);
  }
}

TEST(AcquisitionAvailability, unavailable_services_finish_without_opening)
{
  auto node = std::make_shared<rclcpp::Node>("acquisition_unavailable", "/no_ftp_services");
  auto acquisition = Acquisition::create(node);
  std::optional<Acquisition::Result> result;
  ASSERT_TRUE(acquisition->start(metadata_file("mftp://general.json", 0),
    [&result](Acquisition::Result value) {result = std::move(value);}));
  EXPECT_FALSE(result);
  AcquisitionTestPeer::expire_download(*acquisition);
  ASSERT_TRUE(result);
  EXPECT_NE(result->error.find("services unavailable"), std::string::npos);
  EXPECT_FALSE(result->transport_uncertain);
}

TEST_F(AcquisitionTest, explicit_read_timeout_stops_without_retry_or_close)
{
  read_behavior = ReadBehavior::Timeout;
  start();
  ASSERT_TRUE(spin_until([this]() {return result.has_value();}));
  EXPECT_TRUE(result->transport_uncertain);
  EXPECT_EQ(result->decoder, nullptr);
  EXPECT_EQ(offsets.size(), 1U);
  EXPECT_EQ(opens.size(), 1U);
  EXPECT_TRUE(closes.empty());
}

}  // namespace
}  // namespace px4_events
}  // namespace mavros

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
