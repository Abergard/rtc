#include "chrome_trace.hpp"

#include "gtest/gtest.h"

#include <fstream>
#include <iterator>
#include <string>

namespace rtc
{
namespace ut
{
TEST(chrome_trace_ut, writes_complete_event_trace_file)
{
  const std::string file_name{"chrome_trace_ut.json"};

  chrome_trace::instance().set_enabled(true);
  RTC_TRACE_CLEAR();
  {
    RTC_TRACE_SCOPE_CAT("unit scope", "unit");
  }
  RTC_TRACE_FLUSH(file_name);

  std::ifstream file{file_name};
  const std::string content{std::istreambuf_iterator<char>{file}, std::istreambuf_iterator<char>{}};

  ASSERT_NE(content.find("\"traceEvents\""), std::string::npos);
  ASSERT_NE(content.find("\"ph\":\"X\""), std::string::npos);
  ASSERT_NE(content.find("\"cat\":\"unit\""), std::string::npos);
  ASSERT_NE(content.find("\"name\":\"unit scope\""), std::string::npos);
  ASSERT_NE(content.find("\"ts\":"), std::string::npos);
  ASSERT_NE(content.find("\"dur\":"), std::string::npos);

  RTC_TRACE_CLEAR();
  chrome_trace::instance().set_enabled(false);
}

}  // namespace ut
}  // namespace rtc
