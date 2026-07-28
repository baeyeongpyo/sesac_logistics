#ifndef MENTORPI_GZ_SIM__PALLET_MODEL_FACTORY_HPP_
#define MENTORPI_GZ_SIM__PALLET_MODEL_FACTORY_HPP_

#include <filesystem>
#include <string_view>

#include <gz/math/Pose3.hh>
#include <sdf/Model.hh>

#include "mentorpi_gz_sim/pallet_command.hpp"

namespace mentorpi_gz_sim
{
class PalletModelFactory
{
 public:
  explicit PalletModelFactory(std::filesystem::path templateDir);

  sdf::Model PalletModel(
    std::string_view id, const gz::math::Pose3d & pose) const;
  sdf::Model PayloadModel(
    std::string_view id, PalletKind kind,
    const gz::math::Pose3d & palletPose) const;

 private:
  std::filesystem::path templateDir_;
};
}  // namespace mentorpi_gz_sim

#endif  // MENTORPI_GZ_SIM__PALLET_MODEL_FACTORY_HPP_
