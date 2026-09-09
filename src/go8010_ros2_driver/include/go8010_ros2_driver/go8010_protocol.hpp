#pragma once
#include <array>
#include <cstdint>
#include <cstddef>
#include <vector>
namespace go8010 {
struct Command { double q{},dq{},tau{},kp{},kd{}; };
struct State { double q{},dq{},tau{}; int8_t temperature{}; uint8_t mode{},error{}; bool online{}; };
uint16_t crc_ccitt(const uint8_t*, size_t);
std::array<uint8_t,17> encode(uint8_t id, const Command& c);
bool decode(const uint8_t* data, size_t n, uint8_t expected_id, State& s);
}
