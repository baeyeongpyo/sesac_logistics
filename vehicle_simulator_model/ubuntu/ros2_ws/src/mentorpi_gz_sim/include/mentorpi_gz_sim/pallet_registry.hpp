#ifndef MENTORPI_GZ_SIM__PALLET_REGISTRY_HPP_
#define MENTORPI_GZ_SIM__PALLET_REGISTRY_HPP_

#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Vector4.hh>
#include <gz/sim/Types.hh>

#include "mentorpi_gz_sim/pallet_command.hpp"

namespace mentorpi_gz_sim
{
enum class StableState { Empty, Loaded };
enum class TransitionState { None, Loading, Unloading, Replacing };

struct PalletRecord
{
  std::string id;
  PalletKind kind{PalletKind::Fresh};
  StableState state{StableState::Empty};
  TransitionState transition{TransitionState::None};
  gz::sim::Entity palletEntity{gz::sim::kNullEntity};
  gz::sim::Entity palletLink{gz::sim::kNullEntity};
  gz::sim::Entity payloadEntity{gz::sim::kNullEntity};
  gz::sim::Entity payloadLink{gz::sim::kNullEntity};
};

class PalletRegistry
{
 public:
  PalletRegistry(const gz::math::Vector4d & spawnBounds, double occupiedRadius);

  std::optional<CommandError> ValidateSpawn(
    std::string_view id, const gz::math::Vector2d & position) const;
  std::optional<CommandError> ValidateMutable(
    std::string_view id,
    const gz::math::Vector3d & linear,
    const gz::math::Vector3d & angular) const;
  static bool IsStopped(
    const gz::math::Vector3d & linear,
    const gz::math::Vector3d & angular);

  void Insert(PalletRecord record);
  void Erase(std::string_view id);
  void SetOccupiedPoses(std::vector<gz::math::Vector2d> poses);
  PalletRecord * Find(std::string_view id);
  const PalletRecord * Find(std::string_view id) const;

 private:
  gz::math::Vector4d spawnBounds_;
  double occupiedRadius_;
  std::unordered_map<std::string, PalletRecord> records_;
  std::vector<gz::math::Vector2d> occupiedPoses_;
};
}  // namespace mentorpi_gz_sim

#endif  // MENTORPI_GZ_SIM__PALLET_REGISTRY_HPP_
