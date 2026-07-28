#include "mentorpi_gz_sim/pallet_command.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

namespace mentorpi_gz_sim
{
namespace
{
std::vector<std::string_view> SplitFields(std::string_view input)
{
  std::vector<std::string_view> fields;
  std::size_t start = 0;
  while (start <= input.size()) {
    const auto separator = input.find('|', start);
    if (separator == std::string_view::npos) {
      fields.push_back(input.substr(start));
      break;
    }
    fields.push_back(input.substr(start, separator - start));
    start = separator + 1;
  }
  return fields;
}

ParseResult Error(std::string code, std::string detail)
{
  return {std::nullopt, CommandError{std::move(code), std::move(detail)}};
}

bool IsValidId(std::string_view id)
{
  if (id.empty() || id.size() > 32 || id.front() < 'a' || id.front() > 'z') {
    return false;
  }

  for (const char character : id.substr(1)) {
    const bool is_lowercase = character >= 'a' && character <= 'z';
    const bool is_digit = character >= '0' && character <= '9';
    if (!is_lowercase && !is_digit && character != '_') {
      return false;
    }
  }
  return true;
}

std::optional<PalletKind> ParseKind(std::string_view input)
{
  if (input == "fresh") {
    return PalletKind::Fresh;
  }
  if (input == "normal") {
    return PalletKind::Normal;
  }
  return std::nullopt;
}

std::optional<RequestedState> ParseState(std::string_view input)
{
  if (input == "empty") {
    return RequestedState::Empty;
  }
  if (input == "loaded") {
    return RequestedState::Loaded;
  }
  return std::nullopt;
}

std::optional<double> ParseFiniteDouble(std::string_view input)
{
  const std::string number{input};
  char * end = nullptr;
  errno = 0;
  const double value = std::strtod(number.c_str(), &end);
  if (end != number.c_str() + number.size() || errno == ERANGE || !std::isfinite(value)) {
    return std::nullopt;
  }
  return value;
}

ParseResult ParseSpawn(const std::vector<std::string_view> & fields)
{
  if (fields.size() != 7) {
    return Error("INVALID_FIELD_COUNT", "spawn requires seven fields");
  }
  if (!IsValidId(fields[1])) {
    return Error("INVALID_ID", "id must match ^[a-z][a-z0-9_]{0,31}$");
  }

  const auto kind = ParseKind(fields[2]);
  if (!kind) {
    return Error("INVALID_KIND", "kind must be fresh or normal");
  }
  const auto state = ParseState(fields[3]);
  if (!state) {
    return Error("INVALID_STATE", "state must be empty or loaded");
  }

  const auto x = ParseFiniteDouble(fields[4]);
  const auto y = ParseFiniteDouble(fields[5]);
  const auto yaw = ParseFiniteDouble(fields[6]);
  if (!x || !y || !yaw) {
    return Error("INVALID_NUMBER", "pose values must be finite floating-point numbers");
  }

  return {Command{
      CommandType::Spawn, std::string{fields[1]}, kind, state,
      gz::math::Pose3d{*x, *y, 0.0, 0.0, 0.0, *yaw}},
    std::nullopt};
}

ParseResult ParseStateCommand(const std::vector<std::string_view> & fields)
{
  if (fields.size() != 4) {
    return Error("INVALID_FIELD_COUNT", "state requires four fields");
  }
  if (!IsValidId(fields[1])) {
    return Error("INVALID_ID", "id must match ^[a-z][a-z0-9_]{0,31}$");
  }

  const auto state = ParseState(fields[2]);
  if (!state) {
    return Error("INVALID_STATE", "state must be empty or loaded");
  }
  const auto kind = ParseKind(fields[3]);
  if (!kind) {
    return Error("INVALID_KIND", "kind must be fresh or normal");
  }

  return {Command{CommandType::State, std::string{fields[1]}, kind, state, std::nullopt},
    std::nullopt};
}

ParseResult ParseRemove(const std::vector<std::string_view> & fields)
{
  if (fields.size() != 2) {
    return Error("INVALID_FIELD_COUNT", "remove requires two fields");
  }
  if (!IsValidId(fields[1])) {
    return Error("INVALID_ID", "id must match ^[a-z][a-z0-9_]{0,31}$");
  }
  return {Command{CommandType::Remove, std::string{fields[1]}, std::nullopt, std::nullopt,
      std::nullopt}, std::nullopt};
}
}  // namespace

ParseResult ParseCommand(std::string_view input)
{
  const auto fields = SplitFields(input);
  if (fields.empty() || fields.front().empty()) {
    return Error("UNKNOWN_COMMAND", "command is required");
  }
  if (fields.front() == "spawn") {
    return ParseSpawn(fields);
  }
  if (fields.front() == "state") {
    return ParseStateCommand(fields);
  }
  if (fields.front() == "remove") {
    return ParseRemove(fields);
  }
  if (fields.front() == "list") {
    if (fields.size() != 1) {
      return Error("INVALID_FIELD_COUNT", "list requires one field");
    }
    return {Command{CommandType::List, "", std::nullopt, std::nullopt, std::nullopt},
      std::nullopt};
  }
  return Error("UNKNOWN_COMMAND", "command must be spawn, state, remove, or list");
}

std::string FormatOk(std::string_view command, std::string_view detail)
{
  return "ok|" + std::string{command} + "|" + std::string{detail};
}

std::string FormatError(std::string_view code, std::string_view detail)
{
  return "error|" + std::string{code} + "|" + std::string{detail};
}
}  // namespace mentorpi_gz_sim
