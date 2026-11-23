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

// Time compression (BT_TIME_SCALE): the scaling of a tree's time ports at load. Standard library
// only; also builds without colcon:
//   g++ -std=c++17 -Iinclude test/base/test_time_scale.cpp -lgtest -lgtest_main -pthread

#include <gtest/gtest.h>

#include <string>

#include "koch2/time_scale.hpp"

using koch2::parse_time_scale;
using koch2::scale_bt_xml_times;
using koch2::scale_duration;
using koch2::scale_period_ms;

TEST(TimeScale, ParsesTheFactor)
{
  EXPECT_DOUBLE_EQ(parse_time_scale(nullptr), 1.0);
  EXPECT_DOUBLE_EQ(parse_time_scale(""), 1.0);
  EXPECT_DOUBLE_EQ(parse_time_scale("10"), 10.0);
  EXPECT_DOUBLE_EQ(parse_time_scale("2.5"), 2.5);
  EXPECT_DOUBLE_EQ(parse_time_scale(" 4 "), 4.0);
  EXPECT_DOUBLE_EQ(parse_time_scale("0"), 1.0);       // invalid: unchanged
  EXPECT_DOUBLE_EQ(parse_time_scale("-3"), 1.0);
  EXPECT_DOUBLE_EQ(parse_time_scale("abc"), 1.0);
  EXPECT_DOUBLE_EQ(parse_time_scale("10x"), 1.0);
  EXPECT_DOUBLE_EQ(parse_time_scale("inf"), 1.0);
  EXPECT_DOUBLE_EQ(parse_time_scale("nan"), 1.0);
}

TEST(TimeScale, ScalesDurationsToIntegersOfAtLeastOne)
{
  EXPECT_EQ(scale_duration(30000, 10.0), 3000);
  EXPECT_EQ(scale_duration(1000, 3.0), 333);
  EXPECT_EQ(scale_duration(5, 10.0), 1);       // 0.5 rounds to 1, never 0
  EXPECT_EQ(scale_duration(1, 100.0), 1);
  EXPECT_EQ(scale_duration(0, 10.0), 0);       // no wait stays no wait
  EXPECT_EQ(scale_duration(1234, 1.0), 1234);
  EXPECT_EQ(scale_period_ms(50, 10.0), 5);
  EXPECT_EQ(scale_period_ms(50, 100.0), 1);
  EXPECT_EQ(scale_period_ms(50, 1.0), 50);
}

TEST(TimeScale, KOneLeavesTheXmlUnchanged)
{
  const std::string xml = R"(<root><BehaviorTree ID="T"><Timeout msec="30000"><Sleep msec="5"/></Timeout></BehaviorTree></root>)";
  const auto r = scale_bt_xml_times(xml, 1.0);
  EXPECT_EQ(r.xml, xml);
  EXPECT_EQ(r.scaled, 0);
}

TEST(TimeScale, ScalesTheBuiltInTimePortsOnly)
{
  const std::string xml =
    "<root BTCPP_format=\"4\">\n"
    "  <BehaviorTree ID=\"Main\">\n"
    "    <Timeout msec=\"600000\">\n"
    "      <Delay delay_msec=\"5000\">\n"
    "        <Sequence>\n"
    "          <Sleep msec=\"1000\"/>\n"
    "          <IsTargetStatic min_static_time_sec=\"5\" timeout=\"0.5\"/>\n"
    "          <NavigateTo target=\"kitchen\" timeout=\"300\"/>\n"
    "          <Speak text=\"Wait 30000 ms\" timeout=\"5000\"/>\n"
    "        </Sequence>\n"
    "      </Delay>\n"
    "    </Timeout>\n"
    "  </BehaviorTree>\n"
    "</root>\n";
  const std::string want =
    "<root BTCPP_format=\"4\">\n"
    "  <BehaviorTree ID=\"Main\">\n"
    "    <Timeout msec=\"60000\">\n"
    "      <Delay delay_msec=\"500\">\n"
    "        <Sequence>\n"
    "          <Sleep msec=\"100\"/>\n"
    "          <IsTargetStatic min_static_time_sec=\"5\" timeout=\"0.5\"/>\n"
    "          <NavigateTo target=\"kitchen\" timeout=\"300\"/>\n"
    "          <Speak text=\"Wait 30000 ms\" timeout=\"5000\"/>\n"
    "        </Sequence>\n"
    "      </Delay>\n"
    "    </Timeout>\n"
    "  </BehaviorTree>\n"
    "</root>\n";
  const auto r = scale_bt_xml_times(xml, 10.0);
  EXPECT_EQ(r.xml, want);
  EXPECT_EQ(r.scaled, 3);
  EXPECT_TRUE(r.not_scaled.empty());
}

TEST(TimeScale, ExplicitFormSingleQuotesAndOtherAttributes)
{
  const std::string xml =
    "<Decorator ID=\"Timeout\" name=\"limit\" msec = '20000' ><Action ID='Sleep' msec=\"30\" /></Decorator>"
    "<Control ID=\"Sequence\" msec=\"999\"/>";
  const std::string want =
    "<Decorator ID=\"Timeout\" name=\"limit\" msec = '2000' ><Action ID='Sleep' msec=\"3\" /></Decorator>"
    "<Control ID=\"Sequence\" msec=\"999\"/>";
  const auto r = scale_bt_xml_times(xml, 10.0);
  EXPECT_EQ(r.xml, want);
  EXPECT_EQ(r.scaled, 2);
}

TEST(TimeScale, CommentsCdataScriptsAndBlackboardReferences)
{
  const std::string xml =
    "<?xml version=\"1.0\"?>\n"
    "<!-- <Timeout msec=\"10000\"> in a comment is not scaled -->\n"
    "<Script code=\"done := x > 3\"/>\n"
    "<Timeout msec=\"{limit_ms}\"><Sleep msec=\" 2000 \"/></Timeout>\n"
    "<Delay delay_msec=\"0\"><AlwaysSuccess/></Delay>\n"
    "<TreeNodesModel><Decorator ID=\"Timeout\"><input_port name=\"msec\"/></Decorator></TreeNodesModel>\n";
  const std::string want =
    "<?xml version=\"1.0\"?>\n"
    "<!-- <Timeout msec=\"10000\"> in a comment is not scaled -->\n"
    "<Script code=\"done := x > 3\"/>\n"
    "<Timeout msec=\"{limit_ms}\"><Sleep msec=\"200\"/></Timeout>\n"
    "<Delay delay_msec=\"0\"><AlwaysSuccess/></Delay>\n"
    "<TreeNodesModel><Decorator ID=\"Timeout\"><input_port name=\"msec\"/></Decorator></TreeNodesModel>\n";
  const auto r = scale_bt_xml_times(xml, 10.0);
  EXPECT_EQ(r.xml, want);
  EXPECT_EQ(r.scaled, 2);     // the Sleep and the Delay of 0 (which stays 0)
  ASSERT_EQ(r.not_scaled.size(), 1u);
  EXPECT_EQ(r.not_scaled[0], "Timeout.msec={limit_ms}");
}

TEST(TimeScale, MalformedXmlIsCopiedAsItIs)
{
  const std::string xml = "<root><Timeout msec=\"100><Sleep msec=\"10\"/>";
  const auto r = scale_bt_xml_times(xml, 10.0);
  EXPECT_EQ(r.xml.size() > 0, true);
  const std::string truncated = "<Timeout msec=\"5000";
  EXPECT_EQ(scale_bt_xml_times(truncated, 10.0).xml, truncated);
}
