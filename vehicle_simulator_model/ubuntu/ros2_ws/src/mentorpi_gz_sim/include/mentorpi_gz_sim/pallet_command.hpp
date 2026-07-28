#ifndef MENTORPI_GZ_SIM__PALLET_COMMAND_HPP_
#define MENTORPI_GZ_SIM__PALLET_COMMAND_HPP_

#include <optional>
#include <string>
#include <string_view>

#include <gz/math/Pose3.hh>

namespace mentorpi_gz_sim
{
enum class CommandType { Spawn, State, Remove, List };
enum class PalletKind { Fresh, Normal };
enum class RequestedState { Empty, Loaded };

struct Command
{
  CommandType type;
  std::string id;
  std::optional<PalletKind> kind;
  std::optional<RequestedState> state;
  std::optional<gz::math::Pose3d> pose;
};

struct CommandError
{
  std::string code;
  std::string detail;
};

struct ParseResult
{
  std::optional<Command> command;
  std::optional<CommandError> error;
};

ParseResult ParseCommand(std::string_view input);
std::string FormatOk(std::string_view command, std::string_view detail);
std::string FormatError(std::string_view code, std::string_view detail);
}  // namespace mentorpi_gz_sim

#endif  // MENTORPI_GZ_SIM__PALLET_COMMAND_HPP_
