#include "chrome_trace.hpp"

#include <fstream>
#include <functional>
#include <ostream>
#include <utility>

#if defined(_WIN32)
#include <windows.h>
#else
#include <unistd.h>
#endif

namespace rtc
{
namespace
{
const auto trace_start = std::chrono::steady_clock::now();
}

auto chrome_trace::instance() -> chrome_trace&
{
  static chrome_trace trace;
  return trace;
}

auto chrome_trace::set_output_file(std::string file_name) -> void
{
  std::lock_guard<std::mutex> lock{mutex};
  output_file = std::move(file_name);
}

auto chrome_trace::add_complete_event(const char* name,
                                      const char* category,
                                      std::chrono::steady_clock::time_point start,
                                      std::chrono::steady_clock::time_point end) -> void
{
  complete_event event;
  event.name = name;
  event.category = category;
  event.timestamp_us = timestamp_us(start);
  event.duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  event.process_id = process_id();
  event.thread_id = thread_id();

  std::lock_guard<std::mutex> lock{mutex};
  events.emplace_back(std::move(event));
}

auto chrome_trace::clear() -> void
{
  std::lock_guard<std::mutex> lock{mutex};
  events.clear();
}

auto chrome_trace::flush() -> void
{
  std::string file_name;
  std::vector<complete_event> events_copy;

  {
    std::lock_guard<std::mutex> lock{mutex};
    file_name = output_file;
    events_copy = events;
  }

  write_events(file_name, events_copy);
}

auto chrome_trace::flush(const std::string& file_name) -> void
{
  std::vector<complete_event> events_copy;

  {
    std::lock_guard<std::mutex> lock{mutex};
    events_copy = events;
  }

  write_events(file_name, events_copy);
}

auto chrome_trace::write_events(const std::string& file_name, const std::vector<complete_event>& events) -> void
{
  std::ofstream out{file_name};
  out << "{\n\"traceEvents\":[\n";

  for (std::size_t i{}; i < events.size(); ++i)
  {
    const auto& event = events[i];
    if (i != 0)
      out << ",\n";

    out << "{\"ph\":\"X\",\"cat\":";
    json_escape(out, event.category);
    out << ",\"name\":";
    json_escape(out, event.name);
    out << ",\"pid\":" << event.process_id << ",\"tid\":" << event.thread_id << ",\"ts\":" << event.timestamp_us
        << ",\"dur\":" << event.duration_us << ",\"args\":{}}";
  }

  out << "\n]\n}\n";
}

auto chrome_trace::process_id() noexcept -> std::uint32_t
{
#if defined(_WIN32)
  return static_cast<std::uint32_t>(GetCurrentProcessId());
#else
  return static_cast<std::uint32_t>(getpid());
#endif
}

auto chrome_trace::thread_id() noexcept -> std::uint64_t
{
  return static_cast<std::uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id()));
}

auto chrome_trace::timestamp_us(std::chrono::steady_clock::time_point value) noexcept -> std::uint64_t
{
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(value - trace_start).count());
}

auto chrome_trace::json_escape(std::ostream& out, const std::string& value) -> void
{
  out << '"';
  for (const auto ch : value)
  {
    switch (ch)
    {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << ch;
        break;
    }
  }
  out << '"';
}

chrome_trace_scope::chrome_trace_scope(const char* scope_name, const char* scope_category) noexcept
    : name{scope_name}, category{scope_category}, start{std::chrono::steady_clock::now()}
{
}

chrome_trace_scope::~chrome_trace_scope() noexcept
try
  {
    chrome_trace::instance().add_complete_event(name, category, start, std::chrono::steady_clock::now());
  }
catch (...)
  {
  }

}  // namespace rtc
