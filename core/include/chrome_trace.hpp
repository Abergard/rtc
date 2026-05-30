#pragma once

#include <chrono>
#include <cstdint>
#include <iosfwd>
#include <atomic>
#include <string>
#include <thread>
#include <vector>

namespace rtc
{
class chrome_trace
{
 public:
  static auto instance() -> chrome_trace&;
  ~chrome_trace() noexcept;

  auto set_output_file(std::string file_name) -> void;
  auto add_complete_event(const char* name,
                          const char* category,
                          std::chrono::steady_clock::time_point start,
                          std::chrono::steady_clock::time_point end) -> void;
  auto clear() -> void;
  auto flush() -> void;
  auto flush(const std::string& file_name) -> void;

 private:
  struct complete_event
  {
    std::string name;
    std::string category;
    std::uint64_t timestamp_us{};
    std::uint64_t duration_us{};
    std::uint32_t process_id{};
    std::uint64_t thread_id{};
  };

  struct thread_buffer
  {
    std::vector<complete_event> events;
    thread_buffer* next{};
  };

  chrome_trace() = default;

  auto current_thread_buffer() noexcept -> thread_buffer&;
  static auto process_id() noexcept -> std::uint32_t;
  static auto thread_id() noexcept -> std::uint64_t;
  static auto timestamp_us(std::chrono::steady_clock::time_point value) noexcept -> std::uint64_t;
  static auto write_events(const std::string& file_name, const std::vector<thread_buffer*>& buffers) -> void;
  static auto json_escape(std::ostream& out, const std::string& value) -> void;

  std::atomic<thread_buffer*> buffers{};
  std::string output_file{"rtc_trace.json"};
};

class chrome_trace_scope
{
 public:
  explicit chrome_trace_scope(const char* name, const char* category = "rtc") noexcept;
  ~chrome_trace_scope() noexcept;

  auto operator=(chrome_trace_scope&&) -> chrome_trace_scope& = delete;
  auto operator=(const chrome_trace_scope&) -> chrome_trace_scope& = delete;
  chrome_trace_scope(const chrome_trace_scope&) = delete;
  chrome_trace_scope(chrome_trace_scope&&) = delete;

 private:
  const char* name;
  const char* category;
  std::chrono::steady_clock::time_point start;
};

#define RTC_CHROME_TRACE_COMBINE_TEMP(X, Y) X##Y
#define RTC_CHROME_TRACE_COMBINE(X, Y) RTC_CHROME_TRACE_COMBINE_TEMP(X, Y)

#define RTC_TRACE_SCOPE(NAME) ::rtc::chrome_trace_scope RTC_CHROME_TRACE_COMBINE(__rtc_trace_, __LINE__){NAME}
#define RTC_TRACE_SCOPE_CAT(NAME, CATEGORY) \
  ::rtc::chrome_trace_scope RTC_CHROME_TRACE_COMBINE(__rtc_trace_, __LINE__){NAME, CATEGORY}
#define RTC_TRACE_FUNCTION() RTC_TRACE_SCOPE_CAT(__func__, "function")
#define RTC_TRACE_FLUSH(FILE_NAME) ::rtc::chrome_trace::instance().flush(FILE_NAME)
#define RTC_TRACE_CLEAR() ::rtc::chrome_trace::instance().clear()

}  // namespace rtc
