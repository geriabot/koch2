// Copyright 2026 Rodrigo Pérez-Rodríguez
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef KOCH2__TIME_SCALE_HPP_
#define KOCH2__TIME_SCALE_HPP_

// Time compression for evaluation on test doubles: with BT_TIME_SCALE=k (k > 0, default 1) every
// clock the tree depends on runs k times faster, so that a mission that waits minutes on the test
// doubles takes seconds. BehaviorTree.CPP measures Timeout, Delay and Sleep with the wall clock:
// their time ports are divided by k when the tree is loaded (the tree given or generated is kept
// as it is: only the copy that is built is scaled). The test doubles (dummy_bt_nodes) read the same
// variable and run their own clock k times faster (their duration ports are therefore NOT scaled:
// a port value keeps its meaning and its validation, e.g. min_static_time_sec < 30). The executor
// divides its tick periods and its step timeout by k. k = 1 (or unset) changes nothing.
//
// Dependency-free (standard library only): unit-testable without ROS.

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

namespace koch2
{

/// k of a BT_TIME_SCALE value: a finite number > 0; anything else (unset, empty, invalid) is 1
inline double parse_time_scale(const char * raw)
{
  if (raw == nullptr || *raw == '\0') {
    return 1.0;
  }
  char * end = nullptr;
  const double k = std::strtod(raw, &end);
  if (end == raw || !std::isfinite(k) || k <= 0.0) {
    return 1.0;
  }
  while (*end != '\0' && std::isspace(static_cast<unsigned char>(*end))) {
    ++end;
  }
  return *end == '\0' ? k : 1.0;
}

/// k of the environment variable BT_TIME_SCALE (1 when unset or invalid)
inline double time_scale_from_env()
{
  return parse_time_scale(std::getenv("BT_TIME_SCALE"));
}

/// A duration divided by k, rounded, at least 1 (0 stays 0: "no wait" keeps its meaning)
inline long long scale_duration(long long value, double k)
{
  if (k == 1.0 || value <= 0) {
    return value;
  }
  const long long scaled = std::llround(static_cast<double>(value) / k);
  return scaled < 1 ? 1 : scaled;
}

/// A timer period (ms) divided by k, at least 1 ms
inline int scale_period_ms(int period_ms, double k)
{
  return static_cast<int>(scale_duration(period_ms, k));
}

/// The ports of BehaviorTree.CPP's built-in nodes that are durations measured with the wall clock
/// (node ID -> port, milliseconds): the only time values of a tree that the executor scales
inline const std::map<std::string, std::string> & builtin_time_ports()
{
  static const std::map<std::string, std::string> ports{
    {"Timeout", "msec"}, {"Delay", "delay_msec"}, {"Sleep", "msec"}};
  return ports;
}

struct ScaledXml
{
  std::string xml;                       // the XML with the time ports divided by k
  int scaled = 0;                        // time ports scaled
  std::vector<std::string> not_scaled;   // "Node.port=value" that could not be (e.g. {key})
};

namespace detail
{

inline bool is_name_char(char c)
{
  return std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-' || c == '.' || c == ':';
}

inline bool all_digits(const std::string & s)
{
  if (s.empty() || s.size() > 18) {
    return false;
  }
  for (char c : s) {
    if (!std::isdigit(static_cast<unsigned char>(c))) {
      return false;
    }
  }
  return true;
}

inline std::string trim(const std::string & s)
{
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) {
    return "";
  }
  return s.substr(b, s.find_last_not_of(" \t\r\n") - b + 1);
}

}  // namespace detail

/// The XML of a tree with the time ports of BehaviorTree.CPP's built-in nodes (Timeout msec,
/// Delay delay_msec, Sleep msec; compact form <Timeout msec=...> or explicit form
/// <Decorator ID="Timeout" msec=...>) divided by k (integers, at least 1; 0 stays 0). The rest of
/// the text is copied byte for byte (comments, CDATA, layout, other attributes). A value that is
/// not a literal integer (a blackboard reference {key}) cannot be scaled: it is left as it is and
/// listed in not_scaled. k = 1: the XML unchanged.
inline ScaledXml scale_bt_xml_times(const std::string & xml, double k)
{
  ScaledXml result;
  if (k == 1.0) {
    result.xml = xml;
    return result;
  }
  const auto & ports = builtin_time_ports();
  const std::size_t n = xml.size();
  std::string & out = result.xml;
  out.reserve(n);
  std::size_t i = 0;
  auto copy_until = [&](std::size_t from, const char * marker, std::size_t skip) {
      // Copies [from, end of marker) verbatim and returns the position after it
      std::size_t e = xml.find(marker, from + skip);
      e = (e == std::string::npos) ? n : e + std::char_traits<char>::length(marker);
      out.append(xml, from, e - from);
      return e;
    };
  while (i < n) {
    const std::size_t lt = xml.find('<', i);
    if (lt == std::string::npos) {
      out.append(xml, i, n - i);
      break;
    }
    out.append(xml, i, lt - i);
    if (xml.compare(lt, 4, "<!--") == 0) {
      i = copy_until(lt, "-->", 4);
      continue;
    }
    if (xml.compare(lt, 9, "<![CDATA[") == 0) {
      i = copy_until(lt, "]]>", 9);
      continue;
    }
    if (lt + 1 >= n || xml[lt + 1] == '?' || xml[lt + 1] == '!' || xml[lt + 1] == '/') {
      i = copy_until(lt, ">", 1);       // processing instruction, declaration, end tag
      continue;
    }
    // A start tag: its name, then its attributes (quoted values may hold '>')
    std::size_t p = lt + 1;
    while (p < n && detail::is_name_char(xml[p])) {
      ++p;
    }
    const std::string name = xml.substr(lt + 1, p - lt - 1);
    struct Attr
    {
      std::string name;
      std::size_t begin, end;            // the value, without quotes
    };
    std::vector<Attr> attrs;
    bool malformed = name.empty();
    while (!malformed) {
      while (p < n && std::isspace(static_cast<unsigned char>(xml[p]))) {
        ++p;
      }
      if (p >= n) {
        malformed = true;
        break;
      }
      if (xml[p] == '>' || xml[p] == '/') {
        break;
      }
      const std::size_t a = p;
      while (p < n && detail::is_name_char(xml[p])) {
        ++p;
      }
      const std::string attr = xml.substr(a, p - a);
      while (p < n && std::isspace(static_cast<unsigned char>(xml[p]))) {
        ++p;
      }
      if (attr.empty() || p >= n || xml[p] != '=') {
        malformed = true;
        break;
      }
      ++p;
      while (p < n && std::isspace(static_cast<unsigned char>(xml[p]))) {
        ++p;
      }
      if (p >= n || (xml[p] != '"' && xml[p] != '\'')) {
        malformed = true;
        break;
      }
      const char quote = xml[p];
      const std::size_t vb = p + 1;
      const std::size_t ve = xml.find(quote, vb);
      if (ve == std::string::npos) {
        malformed = true;
        break;
      }
      attrs.push_back({attr, vb, ve});
      p = ve + 1;
    }
    if (malformed) {
      // Not for this function to judge: copied as it is (the XML parser reports it)
      i = copy_until(lt, ">", 1);
      continue;
    }
    std::size_t tag_end = p + 1;
    if (xml[p] == '/') {
      const std::size_t gt = xml.find('>', p);
      tag_end = (gt == std::string::npos) ? n : gt + 1;
    }
    std::string id = name;
    if (name == "Action" || name == "Decorator" || name == "Control" || name == "Condition") {
      id.clear();
      for (const auto & at : attrs) {
        if (at.name == "ID") {
          id = xml.substr(at.begin, at.end - at.begin);
        }
      }
    }
    const auto port = ports.find(id);
    std::size_t copied = lt;
    if (port != ports.end()) {
      for (const auto & at : attrs) {
        if (at.name != port->second) {
          continue;
        }
        const std::string raw = xml.substr(at.begin, at.end - at.begin);
        const std::string value = detail::trim(raw);
        if (detail::all_digits(value)) {
          out.append(xml, copied, at.begin - copied);
          out += std::to_string(scale_duration(std::stoll(value), k));
          copied = at.end;
          ++result.scaled;
        } else {
          result.not_scaled.push_back(id + "." + at.name + "=" + raw);
        }
      }
    }
    out.append(xml, copied, tag_end - copied);
    i = tag_end;
  }
  return result;
}

}  // namespace koch2

#endif  // KOCH2__TIME_SCALE_HPP_
