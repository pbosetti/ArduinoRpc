// SPDX-License-Identifier: Apache-2.0
// Tests for extras/host/tools/rpc_repl/json_value.hpp (docs/PLAN.md, component 7):
// the command-line tokenizer / parse_args(), and the serial_rpc::Value <->
// nlohmann::json conversion. Links nlohmann_json only (not replxx, not
// fmt/cxxopts): this exercises exactly the two things docs/PLAN.md calls
// out as unit-testable in the REPL ("the terminal parts are tested
// manually").
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "json_value.hpp"

#include <cstdint>
#include <string>
#include <vector>

using rpc_repl::ParseError;
using rpc_repl::ParsedCommand;
using rpc_repl::json;
using rpc_repl::json_to_value;
using rpc_repl::parse_args;
using rpc_repl::value_to_json;
using serial_rpc::Value;

// ===========================================================================
// parse_args() / the tokenizer
// ===========================================================================

TEST_CASE("parse_args: empty and whitespace-only lines yield no command") {
  CHECK(parse_args("").command.empty());
  CHECK(parse_args("").args.empty());
  CHECK(parse_args("   ").command.empty());
  CHECK(parse_args("\t  \t").command.empty());
}

TEST_CASE("parse_args: trailing spaces don't produce a phantom trailing token") {
  const ParsedCommand p = parse_args("foo   ");
  CHECK(p.command == "foo");
  CHECK(p.args.empty());
}

TEST_CASE("parse_args: bare numeric/bool/null tokens parse as JSON literals") {
  const ParsedCommand p = parse_args("set_led 13 true");
  REQUIRE(p.command == "set_led");
  REQUIRE(p.args.size() == 2);
  CHECK(p.args[0].is_integer());
  CHECK(p.args[0].as<int>() == 13);
  CHECK(p.args[1].is_bool());
  CHECK(p.args[1].as<bool>() == true);

  const ParsedCommand p2 = parse_args("cfg null -3.5");
  REQUIRE(p2.args.size() == 2);
  CHECK(p2.args[0].is_nil());
  CHECK(p2.args[1].is_double());
  CHECK(p2.args[1].as<double>() == doctest::Approx(-3.5));
}

TEST_CASE("parse_args: a bare token that isn't valid JSON falls back to a plain string") {
  const ParsedCommand p = parse_args("echo hello");
  REQUIRE(p.command == "echo");
  REQUIRE(p.args.size() == 1);
  CHECK(p.args[0].is_string());
  CHECK(p.args[0].as<std::string>() == "hello");
}

TEST_CASE("parse_args: a double-quoted token is always a string, even if JSON-shaped") {
  const ParsedCommand p = parse_args(R"(echo "hi there")");
  REQUIRE(p.args.size() == 1);
  CHECK(p.args[0].is_string());
  CHECK(p.args[0].as<std::string>() == "hi there");

  // Quoted "13" must stay the string "13", not become the integer 13.
  const ParsedCommand p2 = parse_args(R"(echo "13")");
  REQUIRE(p2.args.size() == 1);
  CHECK(p2.args[0].is_string());
  CHECK(p2.args[0].as<std::string>() == "13");
}

TEST_CASE("parse_args: single-quoted tokens work the same way as double-quoted ones") {
  const ParsedCommand p = parse_args("echo 'hi there'");
  REQUIRE(p.args.size() == 1);
  CHECK(p.args[0].as<std::string>() == "hi there");
}

TEST_CASE("parse_args: backslash escapes inside a quoted token") {
  // Command line: echo "a\"b\\c"  ->  arg: a"b\c
  const ParsedCommand p = parse_args("echo \"a\\\"b\\\\c\"");
  REQUIRE(p.args.size() == 1);
  CHECK(p.args[0].as<std::string>() == "a\"b\\c");
}

TEST_CASE("parse_args: backslash escapes a space (or anything else) outside quotes") {
  // Command line: echo foo\ bar  ->  one bare token "foo bar", not valid
  // JSON (unquoted, contains a space), so it falls back to a string.
  const ParsedCommand p = parse_args("echo foo\\ bar");
  REQUIRE(p.args.size() == 1);
  CHECK(p.args[0].as<std::string>() == "foo bar");

  // An escaped bracket never opens a bracket span.
  const ParsedCommand p2 = parse_args("echo foo\\[bar baz");
  REQUIRE(p2.args.size() == 2);
  CHECK(p2.args[0].as<std::string>() == "foo[bar");
  CHECK(p2.args[1].as<std::string>() == "baz");
}

TEST_CASE("parse_args: balanced [...]/{...} spans whitespace as a single token") {
  const ParsedCommand p = parse_args(R"(cfg {"kp": 1.5, "ki": [0, 1]})");
  REQUIRE(p.command == "cfg");
  REQUIRE(p.args.size() == 1);
  REQUIRE(p.args[0].is_map());
  const auto &m = p.args[0].map();
  REQUIRE(m.size() == 2);
  CHECK(m[0].first.as<std::string>() == "kp");
  CHECK(m[0].second.as<double>() == doctest::Approx(1.5));
  CHECK(m[1].first.as<std::string>() == "ki");
  REQUIRE(m[1].second.is_array());
  const auto ki = m[1].second.as<std::vector<Value>>();
  REQUIRE(ki.size() == 2);
  CHECK(ki[0].as<int>() == 0);
  CHECK(ki[1].as<int>() == 1);
}

TEST_CASE("parse_args: a space inside a JSON string within brackets doesn't split the token") {
  const ParsedCommand p = parse_args(R"(cfg {"note": "hi there", "n": 2})");
  REQUIRE(p.args.size() == 1);
  REQUIRE(p.args[0].is_map());
  const auto &m = p.args[0].map();
  REQUIRE(m.size() == 2);
  CHECK(m[0].second.as<std::string>() == "hi there");
  CHECK(m[1].second.as<int>() == 2);
}

TEST_CASE("parse_args: a bracketed token that isn't valid JSON falls back to a plain string") {
  const ParsedCommand p = parse_args("cfg {not valid json}");
  REQUIRE(p.args.size() == 1);
  CHECK(p.args[0].is_string());
  CHECK(p.args[0].as<std::string>() == "{not valid json}");
}

TEST_CASE("parse_args: an array token") {
  const ParsedCommand p = parse_args("set_pixels [1, 2, 3]");
  REQUIRE(p.args.size() == 1);
  REQUIRE(p.args[0].is_array());
  const auto arr = p.args[0].as<std::vector<Value>>();
  REQUIRE(arr.size() == 3);
  CHECK(arr[2].as<int>() == 3);
}

TEST_CASE("parse_args: unbalanced brackets raise a ParseError") {
  CHECK_THROWS_AS(parse_args("cfg {\"a\": 1"), ParseError);   // never closed
  CHECK_THROWS_AS(parse_args("cfg [1, 2"), ParseError);       // never closed
  CHECK_THROWS_AS(parse_args("foo]"), ParseError);            // closing with none open
  CHECK_THROWS_AS(parse_args("cfg {\"a\": [1, 2}"), ParseError); // mismatched
}

TEST_CASE("parse_args: an unterminated quote raises a ParseError") {
  CHECK_THROWS_AS(parse_args("echo \"abc"), ParseError);
  CHECK_THROWS_AS(parse_args("echo 'abc"), ParseError);
}

// ===========================================================================
// Value <-> JSON conversion
// ===========================================================================

TEST_CASE("value_to_json/json_to_value: nil, bool, string round-trip") {
  CHECK(json_to_value(value_to_json(Value())).is_nil());
  CHECK(json_to_value(value_to_json(Value(true))).as<bool>() == true);
  CHECK(json_to_value(value_to_json(Value(false))).as<bool>() == false);
  CHECK(json_to_value(value_to_json(Value(std::string("hello")))).as<std::string>() == "hello");
}

TEST_CASE("value_to_json/json_to_value: signed/unsigned integers keep their JSON number kind") {
  const json ji = value_to_json(Value(int64_t{-42}));
  CHECK(ji.is_number_integer());
  CHECK(json_to_value(ji).is_int());
  CHECK(json_to_value(ji).as<int64_t>() == -42);

  const json ju = value_to_json(Value(uint64_t{42}));
  CHECK(ju.is_number_unsigned());
  CHECK(json_to_value(ju).is_uint());
  CHECK(json_to_value(ju).as<uint64_t>() == 42);
}

TEST_CASE("value_to_json/json_to_value: double round-trips") {
  const Value v(3.5);
  CHECK(json_to_value(value_to_json(v)).as<double>() == doctest::Approx(3.5));
}

TEST_CASE("value_to_json/json_to_value: bin round-trips through the {\"$bin\": hex} convention") {
  const Value::Bin bin{0x00, 0x01, 0xAB, 0xFF};
  const json j = value_to_json(Value(bin));
  REQUIRE(j.is_object());
  REQUIRE(j.contains("$bin"));
  CHECK(j["$bin"].get<std::string>() == "0001abff");

  const Value back = json_to_value(j);
  REQUIRE(back.is_bin());
  CHECK(back.as<Value::Bin>() == bin);
}

TEST_CASE("value_to_json/json_to_value: array round-trips, preserving order") {
  Value::Array arr;
  arr.emplace_back(int64_t{1});
  arr.emplace_back(std::string("two"));
  arr.emplace_back(true);
  const json j = value_to_json(Value(arr));
  REQUIRE(j.is_array());
  REQUIRE(j.size() == 3);

  const Value back = json_to_value(j);
  const auto v = back.as<std::vector<Value>>();
  REQUIRE(v.size() == 3);
  CHECK(v[0].as<int64_t>() == 1);
  CHECK(v[1].as<std::string>() == "two");
  CHECK(v[2].as<bool>() == true);
}

TEST_CASE("value_to_json/json_to_value: a string-keyed map round-trips, preserving key order") {
  Value::Map m;
  m.emplace_back(Value(std::string("z")), Value(int64_t{1}));
  m.emplace_back(Value(std::string("a")), Value(int64_t{2}));
  const json j = value_to_json(Value(m));
  REQUIRE(j.is_object());
  // ordered_json: dump() reflects insertion order, not alphabetical order.
  CHECK(j.dump() == R"({"z":1,"a":2})");

  const Value back = json_to_value(j);
  REQUIRE(back.is_map());
  const auto &bm = back.map();
  REQUIRE(bm.size() == 2);
  CHECK(bm[0].first.as<std::string>() == "z");
  CHECK(bm[1].first.as<std::string>() == "a");
}

TEST_CASE("value_to_json: a non-string map key is stringified") {
  Value::Map m;
  m.emplace_back(Value(int64_t{1}), Value(std::string("one")));
  const json j = value_to_json(Value(m));
  REQUIRE(j.is_object());
  REQUIRE(j.contains("1"));
  CHECK(j["1"].get<std::string>() == "one");
}

TEST_CASE("json_to_value: a JSON object without a lone \"$bin\" key decodes as an ordinary map") {
  const json j = json::parse(R"({"$bin": "not-hex!"})"); // odd chars: not valid hex
  const Value v = json_to_value(j);
  REQUIRE(v.is_map());
  CHECK(v.map().size() == 1);

  const json j2 = json::parse(R"({"$bin": "ab", "extra": 1})"); // two keys: not the bin shape
  const Value v2 = json_to_value(j2);
  REQUIRE(v2.is_map());
  CHECK(v2.map().size() == 2);
}
