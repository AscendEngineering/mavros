/*
 * Copyright 2026 Sean Hickey.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Bounded general/event metadata retrieval and validation
 * @file acquisition.cpp
 * @author Sean Hickey
 */

#include "acquisition.hpp"

#include <algorithm>
#include <cerrno>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

#include "mavros/utils.hpp"

namespace mavros
{
namespace px4_events
{

std::shared_ptr<Acquisition> Acquisition::create(
  const rclcpp::Node::SharedPtr & node, Decoder::ErrorCallback report_error)
{
  auto acquisition = std::shared_ptr<Acquisition>(new Acquisition(node, std::move(report_error)));
  acquisition->initialize();
  return acquisition;
}

Acquisition::Acquisition(
  const rclcpp::Node::SharedPtr & node, Decoder::ErrorCallback report_error)
: node_(node), report_error_(std::move(report_error))
{}

void Acquisition::initialize()
{
  group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
#ifdef USE_OLD_RMW_QOS
  auto qos = rmw_qos_profile_services_default;
#else
  auto qos = rclcpp::ServicesQoS();
#endif
  open_ = node_->create_client<Open>("ftp/open", qos, group_);
  read_ = node_->create_client<Read>("ftp/read", qos, group_);
  close_ = node_->create_client<Close>("ftp/close", qos, group_);
  const std::weak_ptr<Acquisition> weak = shared_from_this();
  timer_ = node_->create_wall_timer(std::chrono::milliseconds(250), [weak]() {
        if (const auto self = weak.lock()) {
          self->tick(Clock::now());
        }
    }, group_);
  timer_->cancel();
}

Acquisition::~Acquisition()
{
  if (timer_) {
    timer_->cancel();
  }
  clear_pending_locked();
}

void Acquisition::clear_pending_locked()
{
  if (pending_) {
    switch (phase_) {
      case Phase::Open: open_->remove_pending_request(*pending_); break;
      case Phase::Read: read_->remove_pending_request(*pending_); break;
      case Phase::Close: close_->remove_pending_request(*pending_); break;
      default: break;
    }
    pending_.reset();
  }
}

bool Acquisition::start(const MetadataFile & general, Callback callback)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (phase_ != Phase::Idle || completion_ || !callback) {
      return false;
    }
    attempts_ = 1;
    general_ = general;
    callback_ = std::move(callback);
    file_ = File::General;
    try {
      begin_file_locked(general_);
      timer_->reset();
    } catch (const std::exception & ex) {
      finish_locked({nullptr, ex.what(), false});
    } catch (...) {
      finish_locked({nullptr, "unknown metadata acquisition setup exception", false});
    }
  }
  tick(Clock::now());
  return true;
}

void Acquisition::begin_file_locked(const MetadataFile & file)
{
  opened_ = false;
  busy_retried_ = false;
  retryable_ = false;
  bytes_.clear();
  error_.clear();
  current_ = metadata_file(file.uri, file.crc);
  if (current_.ftp_path.size() > 238) {
    throw std::runtime_error("metadata FTP path exceeds supported length");
  }
  const auto & path = current_.ftp_path;
  limit_ = path.size() >= 3 && path.substr(path.size() - 3) == ".xz" ?
    Decoder::MAX_COMPRESSED_BYTES : Decoder::MAX_JSON_BYTES;
  phase_ = Phase::Services;
  deadline_ = Clock::now() + WAIT_TIMEOUT;
}

void Acquisition::cancel()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (phase_ != Phase::Idle) {
      if (opened_) {
        RCLCPP_WARN(node_->get_logger(), "PX4 metadata session cleanup skipped after cancellation");
      }
      finish_locked({nullptr, "metadata acquisition cancelled", false});
    }
  }
  deliver();
}

void Acquisition::tick(Clock::time_point now)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      if (phase_ == Phase::Retry && now >= next_request_) {
        file_ = File::General;
        begin_file_locked(general_);
      }
      if (phase_ == Phase::Services) {
        if (open_->service_is_ready() && read_->service_is_ready() && close_->service_is_ready()) {
          phase_ = Phase::Open;
          dispatch_locked();
        } else if (now >= deadline_) {
          stop_locked("metadata FTP services unavailable", false);
        }
      } else if (pending_) {
        if (now >= deadline_) {
          stop_locked("metadata FTP service response timeout", true);
        }
      } else if (phase_ != Phase::Idle && phase_ != Phase::Retry && now >= next_request_) {
        dispatch_locked();
      }
    } catch (const std::exception & ex) {
      stop_locked(ex.what(), true);
    } catch (...) {
      stop_locked("unknown metadata FTP exception", true);
    }
  }
  deliver();
}

template<class Service>
void Acquisition::send_locked(
  const std::shared_ptr<rclcpp::Client<Service>> & client,
  const std::shared_ptr<typename Service::Request> & request,
  void (Acquisition::* handler)(const typename Service::Response &))
{
  const auto token = ++serial_;
  deadline_ = Clock::now() + WAIT_TIMEOUT;
  const std::weak_ptr<Acquisition> weak = shared_from_this();
  const auto future = client->async_send_request(
    request, [weak, token, handler](typename rclcpp::Client<Service>::SharedFuture response) {
      if (const auto self = weak.lock()) {
        {
          std::lock_guard<std::mutex> lock(self->mutex_);
          if (!self->pending_ || token != self->serial_) {
            return;
          }
          self->clear_pending_locked();
          try {
            (self.get()->*handler)(*response.get());
          } catch (const std::exception & ex) {
            self->stop_locked(ex.what(), true);
          } catch (...) {
            self->stop_locked("unknown metadata FTP response exception", true);
          }
        }
        self->deliver();
      }
    });
  pending_ = future.request_id;
}

void Acquisition::dispatch_locked()
{
  if (phase_ == Phase::Open) {
    auto request = std::make_shared<Open::Request>();
    request->file_path = current_.ftp_path;
    request->mode = Open::Request::MODE_READ;
    send_locked<Open>(open_, request, &Acquisition::open_reply_locked);
  } else if (phase_ == Phase::Read) {
    auto request = std::make_shared<Read::Request>();
    request->file_path = current_.ftp_path;
    request->offset = bytes_.size();
    requested_ = std::min(CHUNK_SIZE, size_ - bytes_.size());
    request->size = requested_;
    send_locked<Read>(read_, request, &Acquisition::read_reply_locked);
  } else if (phase_ == Phase::Close) {
    auto request = std::make_shared<Close::Request>();
    request->file_path = current_.ftp_path;
    send_locked<Close>(close_, request, &Acquisition::close_reply_locked);
  }
}

bool Acquisition::failed_locked(bool success, int error)
{
  if (success) {
    busy_retried_ = false;
    return false;
  }
  if (error == EBUSY && !busy_retried_) {
    busy_retried_ = true;
    next_request_ = Clock::now() + BUSY_BACKOFF;
  } else {
    if (phase_ != Phase::Close) {
      retryable_ = error == EBUSY || error == EIO;
    }
    stop_locked(
      "metadata FTP operation failed, errno=" + std::to_string(error),
      error == ETIMEDOUT || error == EILSEQ || error == EBADE ||
      error == EBADSLT || error == EBADRQC);
  }
  return true;
}

void Acquisition::open_reply_locked(const Open::Response & response)
{
  if (failed_locked(response.success, response.r_errno)) {
    return;
  }
  opened_ = true;
  size_ = response.size;
  if (size_ == 0 || size_ > limit_) {
    stop_locked("metadata FTP file is empty or exceeds size limit", false);
    return;
  }
  bytes_.reserve(size_);
  phase_ = Phase::Read;
  dispatch_locked();
}

void Acquisition::read_reply_locked(const Read::Response & response)
{
  if (failed_locked(response.success, response.r_errno)) {
    return;
  }
  if (response.data.empty() || response.data.size() > requested_ ||
    response.data.size() > size_ - bytes_.size())
  {
    stop_locked("metadata FTP read made no progress or returned excess data", false);
    return;
  }
  bytes_.append(reinterpret_cast<const char *>(response.data.data()), response.data.size());
  if (bytes_.size() == size_) {
    phase_ = Phase::Close;
  }
  dispatch_locked();  // No pacing: next read starts directly from this response callback.
}

void Acquisition::close_reply_locked(const Close::Response & response)
{
  if (!failed_locked(response.success, response.r_errno)) {
    opened_ = false;
    file_finished_locked();
  }
}

void Acquisition::stop_locked(const std::string & reason, bool uncertain)
{
  if (phase_ == Phase::Idle) {
    return;
  }
  if (phase_ == Phase::Close) {
    file_finished_locked(reason, uncertain);
    return;
  }
  error_ = reason;
  clear_pending_locked();
  if (opened_ && !uncertain) {
    busy_retried_ = false;
    phase_ = Phase::Close;
    dispatch_locked();
  } else {
    file_finished_locked(opened_ ? "session cleanup skipped after uncertain failure" : "",
          uncertain);
  }
}

void Acquisition::file_finished_locked(const std::string & close_error, bool uncertain)
{
  clear_pending_locked();
  if (!close_error.empty()) {
    RCLCPP_WARN(node_->get_logger(), "PX4 metadata FTP cleanup incomplete: %s",
          close_error.c_str());
  }
  const auto now = Clock::now();
  if (!error_.empty()) {
    const auto error = error_;
    fail_locked(error, retryable_, uncertain, now);
    return;
  }
  if (file_ == File::General && uncertain) {
    fail_locked("general metadata close unresolved", false, true, now);
    return;
  }
  const bool compressed = current_.ftp_path.size() >= 3 &&
    current_.ftp_path.substr(current_.ftp_path.size() - 3) == ".xz";
  try {
    // General CRC covers uncompressed JSON; event CRC covers transferred bytes.
    if (file_ == File::Events && metadata_crc32(bytes_) != current_.crc) {
      fail_locked("event metadata checksum mismatch", true, uncertain, now);
      return;
    }
    const auto json = Decoder::unpack_metadata(bytes_, compressed);
    if (file_ == File::General) {
      if (metadata_crc32(json) != current_.crc) {
        fail_locked("general metadata checksum mismatch", true, false, now);
        return;
      }
      const auto events = event_metadata_file(json);
      file_ = File::Events;
      begin_file_locked(events);
    } else {
      auto decoder = std::make_unique<Decoder>(report_error_);
      const auto error = decoder->load_json(json);
      if (!error.empty()) {
        throw std::runtime_error(error);
      }
      finish_locked({std::move(decoder), {}, uncertain});
    }
  } catch (const std::exception & ex) {
    fail_locked(ex.what(), false, uncertain, now);
  } catch (...) {
    fail_locked("unknown metadata validation exception", false, uncertain, now);
  }
}

void Acquisition::fail_locked(
  const std::string & reason, bool retryable, bool uncertain, Clock::time_point now)
{
  if (retryable && !uncertain && attempts_ < 3) {
    const auto backoff = std::chrono::seconds(attempts_ == 1 ? 2 : 5);
    ++attempts_;
    bytes_.clear();
    error_.clear();
    phase_ = Phase::Retry;
    next_request_ = now + backoff;
    RCLCPP_WARN(node_->get_logger(), "PX4 event metadata attempt failed: %s; retrying",
      reason.c_str());
    return;
  }
  finish_locked({nullptr, reason, uncertain});
}

void Acquisition::finish_locked(Result result)
{
  clear_pending_locked();
  phase_ = Phase::Idle;
  timer_->cancel();
  bytes_.clear();
  error_.clear();
  if (!result.error.empty()) {
    RCLCPP_WARN(node_->get_logger(), "PX4 event acquisition failed: %s; retaining raw logging",
      result.error.c_str());
  }
  completion_ = Completion{std::move(callback_), std::move(result)};
}

void Acquisition::deliver()
{
  std::optional<Completion> completion;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    completion = std::move(completion_);
    completion_.reset();
  }
  if (completion) {
    try {
      completion->callback(std::move(completion->result));
    } catch (...) {
      RCLCPP_ERROR(node_->get_logger(), "PX4 event acquisition completion callback failed");
    }
  }
}

}  // namespace px4_events
}  // namespace mavros
