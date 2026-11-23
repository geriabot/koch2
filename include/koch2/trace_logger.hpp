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

#ifndef KOCH2__TRACE_LOGGER_HPP_
#define KOCH2__TRACE_LOGGER_HPP_

#include <chrono>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/loggers/abstract_logger.h"

namespace koch2
{

/// The trace of a tree's execution, for test oracles: one JSON line per leaf node (action or
/// condition) when it starts and when it ends, with its input ports ({key, value}: the key it reads,
/// if any, and the value) and, at the end, the values its output ports wrote.
/// Enabled when the environment variable BT_TRACE_FILE names a file (lines are appended).
/// Under time compression (BT_TIME_SCALE=k != 1) every line also carries "k": its times ("t", wall
/// clock, ms) then stand for k times as much simulated time (the trace oracle scales its tick window).
class TraceLogger : public BT::StatusChangeLogger
{
public:
  TraceLogger(
    const BT::Tree & tree, const std::string & path, const std::string & label,
    double time_scale = 1.0)
  : BT::StatusChangeLogger(), out_(path, std::ios::app), label_(label), time_scale_(time_scale)
  {
    enableTransitionToIdle(true);   // to see halts (RUNNING -> IDLE)
    subscribeToTreeChanges(tree.rootNode());
  }

  ~TraceLogger() override
  {
    unsubscribeFromTreeChanges();
  }

  void callback(
    BT::Duration, const BT::TreeNode & node, BT::NodeStatus prev,
    BT::NodeStatus status) override
  {
    if (node.type() != BT::NodeType::ACTION && node.type() != BT::NodeType::CONDITION) {
      return;
    }
    if (prev == BT::NodeStatus::IDLE && status != BT::NodeStatus::IDLE) {
      write(node, "start", status, false);
    }
    if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
      write(node, "end", status, true);
    }
    if (prev == BT::NodeStatus::RUNNING && status == BT::NodeStatus::IDLE) {
      write(node, "halt", status, false);   // interrupted by a reactive parent or a timeout
    }
  }

  void flush() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    out_.flush();
  }

private:
  static std::string escape(const std::string & s)
  {
    std::string r;
    for (char c : s) {
      switch (c) {
        case '"': r += "\\\""; break;
        case '\\': r += "\\\\"; break;
        case '\n': r += "\\n"; break;
        case '\r': r += "\\r"; break;
        case '\t': r += "\\t"; break;
        default:
          if (static_cast<unsigned char>(c) < 0x20) {
            r += ' ';
          } else {
            r += c;
          }
      }
    }
    return r;
  }

  /// A text with "{key}" placeholders inside (e.g. "Hello {name}"): the text with the blackboard
  /// values substituted, and the keys it reads (O3: templated speech must be visible to the oracle).
  static std::string resolve_template(
    const BT::TreeNode & node, const std::string & text, std::vector<std::string> & keys)
  {
    std::string out;
    std::size_t pos = 0;
    while (pos < text.size()) {
      const auto open = text.find('{', pos);
      const auto close = open == std::string::npos ? std::string::npos : text.find('}', open);
      if (open == std::string::npos || close == std::string::npos) {
        out += text.substr(pos);
        break;
      }
      out += text.substr(pos, open - pos);
      const std::string key = text.substr(open + 1, close - open - 1);
      keys.push_back(key);
      out += value_of(node, "{" + key + "}");
      pos = close + 1;
    }
    return out;
  }

  static std::string value_of(const BT::TreeNode & node, const std::string & remap)
  {
    BT::StringView stripped;
    if (!BT::TreeNode::isBlackboardPointer(remap, &stripped)) {
      return remap;
    }
    const auto & bb = node.config().blackboard;
    if (!bb) {
      return "";
    }
    try {
      auto any = bb->getAnyLocked(std::string(stripped));
      if (!any || any->empty()) {
        return "<unset>";
      }
      if (auto as_string = any->tryCast<std::string>()) {
        return as_string.value();
      }
      if (auto as_double = any->tryCast<double>()) {
        std::ostringstream os;
        os << as_double.value();
        return os.str();
      }
      return "<" + BT::demangle(any->type()) + ">";
    } catch (...) {
      return "<unreadable>";
    }
  }

  void write(const BT::TreeNode & node, const char * event, BT::NodeStatus status, bool outputs)
  {
    std::ostringstream line;
    const auto t = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    line << "{\"t\":" << t;
    if (time_scale_ != 1.0) {
      line << ",\"k\":" << time_scale_;
    }
    line << ",\"tree\":\"" << escape(label_) << "\",\"event\":\"" << event
         << "\",\"node\":\"" << escape(node.registrationName()) << "\",\"name\":\""
         << escape(node.name()) << "\",\"status\":\"" << BT::toStr(status) << "\",\"inputs\":{";
    bool first = true;
    for (const auto & [port, remap] : node.config().input_ports) {
      BT::StringView key;
      const bool is_key = BT::TreeNode::isBlackboardPointer(remap, &key);
      std::vector<std::string> embedded;
      const std::string value = is_key ? value_of(node, remap) :
        (remap.find('{') != std::string::npos ? resolve_template(node, remap, embedded) : remap);
      line << (first ? "" : ",") << "\"" << escape(port) << "\":{\"key\":\""
           << escape(is_key ? std::string(key) : "") << "\",\"value\":\""
           << escape(value) << "\"";
      if (!embedded.empty()) {
        line << ",\"keys\":[";
        for (std::size_t i = 0; i < embedded.size(); ++i) {
          line << (i ? "," : "") << "\"" << escape(embedded[i]) << "\"";
        }
        line << "]";
      }
      line << "}";
      first = false;
    }
    line << "}";
    if (outputs) {
      line << ",\"outputs\":{";
      first = true;
      for (const auto & [port, remap] : node.config().output_ports) {
        BT::StringView key;
        const bool is_key = BT::TreeNode::isBlackboardPointer(remap, &key);
        line << (first ? "" : ",") << "\"" << escape(port) << "\":{\"key\":\""
             << escape(is_key ? std::string(key) : remap) << "\",\"value\":\""
             << escape(is_key ? value_of(node, remap) : "") << "\"}";
        first = false;
      }
      line << "}";
    }
    line << "}\n";
    std::lock_guard<std::mutex> lock(mutex_);
    out_ << line.str();
    out_.flush();
  }

  std::ofstream out_;
  std::string label_;
  double time_scale_{1.0};
  std::mutex mutex_;
};

}  // namespace koch2

#endif  // KOCH2__TRACE_LOGGER_HPP_
