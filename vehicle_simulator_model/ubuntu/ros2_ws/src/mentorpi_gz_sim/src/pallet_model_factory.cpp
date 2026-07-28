#include "mentorpi_gz_sim/pallet_model_factory.hpp"

#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

#include <sdf/Root.hh>

namespace mentorpi_gz_sim
{
namespace
{
std::string ReadTemplate(const std::filesystem::path & path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("MODEL_TEMPLATE_INVALID: unable to read " + path.string());
  }
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

void ReplaceAll(std::string & value, std::string_view token, std::string_view replacement)
{
  std::size_t offset = 0;
  while ((offset = value.find(token, offset)) != std::string::npos) {
    value.replace(offset, token.size(), replacement);
    offset += replacement.size();
  }
}

std::string PoseString(const gz::math::Pose3d & pose)
{
  std::ostringstream output;
  output << std::setprecision(17)
         << pose.Pos().X() << ' ' << pose.Pos().Y() << ' ' << pose.Pos().Z() << ' '
         << pose.Rot().Roll() << ' ' << pose.Rot().Pitch() << ' ' << pose.Rot().Yaw();
  return output.str();
}

sdf::Model ParseModel(std::string value)
{
  static const std::regex kUnresolvedToken("@[A-Z_]+@");
  if (std::regex_search(value, kUnresolvedToken)) {
    throw std::runtime_error("MODEL_TEMPLATE_INVALID: unresolved template token");
  }

  sdf::Root root;
  const auto errors = root.LoadSdfString(value);
  if (!errors.empty() || root.Model() == nullptr) {
    throw std::runtime_error("MODEL_TEMPLATE_INVALID");
  }
  return *root.Model();
}

sdf::Model BuildModel(
  const std::filesystem::path & templatePath,
  std::string_view id,
  std::string_view poseToken,
  const gz::math::Pose3d & pose)
{
  auto templateSdf = ReadTemplate(templatePath);
  ReplaceAll(templateSdf, "@PALLET_ID@", id);
  ReplaceAll(templateSdf, poseToken, PoseString(pose));
  return ParseModel(std::move(templateSdf));
}
}  // namespace

PalletModelFactory::PalletModelFactory(std::filesystem::path templateDir)
: templateDir_(std::move(templateDir))
{
}

sdf::Model PalletModelFactory::PalletModel(
  std::string_view id,
  const gz::math::Pose3d & pose) const
{
  return BuildModel(templateDir_ / "pallet.sdf.in", id, "@POSE@", pose);
}

sdf::Model PalletModelFactory::PayloadModel(
  std::string_view id,
  PalletKind kind,
  const gz::math::Pose3d & palletPose) const
{
  const auto payloadPose = palletPose * gz::math::Pose3d(0, 0, 0.030, 0, 0, 0);
  const auto templateName = kind == PalletKind::Fresh ?
    "payload_fresh.sdf.in" : "payload_normal.sdf.in";
  return BuildModel(templateDir_ / templateName, id, "@PAYLOAD_POSE@", payloadPose);
}
}  // namespace mentorpi_gz_sim
