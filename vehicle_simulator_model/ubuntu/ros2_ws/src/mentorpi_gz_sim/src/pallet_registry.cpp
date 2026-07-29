#include "mentorpi_gz_sim/pallet_registry.hpp"

#include <utility>

namespace mentorpi_gz_sim
{
namespace
{
constexpr double kLinearStopThreshold = 0.02;
constexpr double kAngularStopThreshold = 0.05;

CommandError Error(std::string code, std::string detail)
{
  return {std::move(code), std::move(detail)};
}
}  // namespace

PalletRegistry::PalletRegistry(
  const gz::math::Vector4d & spawnBounds,
  double occupiedRadius)
: spawnBounds_(spawnBounds), occupiedRadius_(occupiedRadius)
{
}

std::optional<CommandError> PalletRegistry::ValidateSpawn(
  std::string_view id,
  const gz::math::Vector2d & position) const
{
  if (Find(id) != nullptr) {
    return Error("DUPLICATE_ID", "pallet id is already registered");
  }

  if (position.X() < spawnBounds_.X() || position.X() > spawnBounds_.Y() ||
    position.Y() < spawnBounds_.Z() || position.Y() > spawnBounds_.W())
  {
    return Error("OUT_OF_BOUNDS", "spawn pose is outside warehouse bounds");
  }

  const double radiusSquared = occupiedRadius_ * occupiedRadius_;
  for (const auto & occupiedPose : occupiedPoses_) {
    if ((position - occupiedPose).SquaredLength() <= radiusSquared) {
      return Error("SPAWN_POSE_OCCUPIED", "spawn pose is occupied");
    }
  }
  return std::nullopt;
}

std::optional<CommandError> PalletRegistry::ValidateMutable(
  std::string_view id,
  const gz::math::Vector3d & linear,
  const gz::math::Vector3d & angular) const
{
  if (Find(id) == nullptr) {
    return Error("NOT_FOUND", "pallet id is not registered");
  }
  if (!IsStopped(linear, angular)) {
    return Error("PALLET_NOT_STOPPED", "pallet must be stopped before state changes");
  }
  return std::nullopt;
}

bool PalletRegistry::IsStopped(
  const gz::math::Vector3d & linear,
  const gz::math::Vector3d & angular)
{
  return linear.Length() <= kLinearStopThreshold &&
         angular.Length() <= kAngularStopThreshold;
}

void PalletRegistry::Insert(PalletRecord record)
{
  records_[record.id] = std::move(record);
}

void PalletRegistry::Erase(std::string_view id)
{
  records_.erase(std::string(id));
}

void PalletRegistry::SetOccupiedPoses(std::vector<gz::math::Vector2d> poses)
{
  occupiedPoses_ = std::move(poses);
}

PalletRecord * PalletRegistry::Find(std::string_view id)
{
  const auto iterator = records_.find(std::string(id));
  return iterator == records_.end() ? nullptr : &iterator->second;
}

const PalletRecord * PalletRegistry::Find(std::string_view id) const
{
  const auto iterator = records_.find(std::string(id));
  return iterator == records_.end() ? nullptr : &iterator->second;
}
}  // namespace mentorpi_gz_sim
