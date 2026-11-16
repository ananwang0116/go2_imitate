// Events in the same format as go2_common/events.py: "[mot] NAME key=value ...", on /mot_event and in <run>/events.log.
// An event is one decision or state change: never call this periodically (spec P7).
#pragma once
#include <fcntl.h>
#include <unistd.h>

#include <cmath>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>
#include <utility>
#include <vector>

namespace go2_nav {

inline std::string fmt(bool v) { return v ? "true" : "false"; }
inline std::string fmt(int v) { return std::to_string(v); }
inline std::string fmt(double v) {
  if (!std::isfinite(v)) return std::isnan(v) ? "nan" : (v > 0 ? "inf" : "-inf");
  char b[64];
  std::snprintf(b, sizeof b, "%.3f", v);
  std::string s = b;
  while (!s.empty() && s.back() == '0') s.pop_back();
  if (!s.empty() && s.back() == '.') s += '0';
  return s;
}
inline std::string fmt(const std::string& v) { return (v.empty() || v.find(' ') != std::string::npos) ? "\"" + v + "\"" : v; }
inline std::string fmt(const char* v) { return fmt(std::string(v)); }

using Fields = std::vector<std::pair<std::string, std::string>>;

inline std::string event_text(const std::string& layer, const std::string& name, const Fields& f) {
  std::string t = "[" + layer + "] " + name;
  for (const auto& kv : f) t += " " + kv.first + "=" + kv.second;
  return t;
}

// One write() on an O_APPEND file: lines of different processes do not mix.
inline void append_event_line(const std::string& text, double ros_time_s) {
  const char* d = std::getenv("GO2_RUN_DIR");
  if (!d || !*d) return;
  const auto now = std::chrono::system_clock::now();
  const std::time_t tt = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&tt, &tm);
  char wall[40];
  std::strftime(wall, sizeof wall, "%Y-%m-%d %H:%M:%S", &tm);
  const int ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000);
  char line[1024];
  const int n = std::snprintf(line, sizeof line, "%s.%03d t=%.3f %s\n", wall, ms, ros_time_s, text.c_str());
  const std::string path = std::string(d) + "/events.log";
  const int fd = ::open(path.c_str(), O_WRONLY | O_APPEND | O_CREAT, 0644);
  if (fd >= 0) {
    if (n > 0) (void)!::write(fd, line, std::min<size_t>(n, sizeof line - 1));
    ::close(fd);
  }
}

}  // namespace go2_nav
