#include <gtest/gtest.h>

#include <filesystem>
#include <stdexcept>
#include <string>

#include <sdf/Link.hh>
#include <sdf/Material.hh>
#include <sdf/Visual.hh>

#include "mentorpi_gz_sim/pallet_model_factory.hpp"
#include "mentorpi_gz_sim/pallet_registry.hpp"

namespace mentorpi_gz_sim
{
TEST(PalletRegistry, RejectsDuplicateOutOfBoundsAndOccupiedSpawn)
{
  PalletRegistry registry({-4.5, 4.5, -3.5, 3.5}, 0.20);
  EXPECT_FALSE(registry.ValidateSpawn("pallet_01", {-2.5, -2.8}).has_value());

  PalletRecord record;
  record.id = "pallet_01";
  registry.Insert(record);

  EXPECT_EQ(registry.ValidateSpawn("pallet_01", {0, 0})->code, "DUPLICATE_ID");
  EXPECT_EQ(registry.ValidateSpawn("pallet_02", {5, 0})->code, "OUT_OF_BOUNDS");

  registry.SetOccupiedPoses({gz::math::Vector2d(-2.5, -2.8)});
  EXPECT_EQ(
    registry.ValidateSpawn("pallet_02", {-2.5, -2.8})->code,
    "SPAWN_POSE_OCCUPIED");
}

TEST(PalletRegistry, UsesConfiguredStopThresholds)
{
  EXPECT_TRUE(PalletRegistry::IsStopped({0.02, 0, 0}, {0, 0, 0.05}));
  EXPECT_FALSE(PalletRegistry::IsStopped({0.0201, 0, 0}, {0, 0, 0}));
  EXPECT_FALSE(PalletRegistry::IsStopped({0, 0, 0}, {0, 0, 0.0501}));
}

TEST(PalletRegistry, ReportsMutableValidationErrors)
{
  PalletRegistry registry({-4.5, 4.5, -3.5, 3.5}, 0.20);
  EXPECT_EQ(
    registry.ValidateMutable("missing", {0, 0, 0}, {0, 0, 0})->code,
    "NOT_FOUND");

  PalletRecord record;
  record.id = "pallet_01";
  registry.Insert(record);

  EXPECT_EQ(
    registry.ValidateMutable("pallet_01", {0.0201, 0, 0}, {0, 0, 0})->code,
    "PALLET_NOT_STOPPED");
  EXPECT_FALSE(
    registry.ValidateMutable("pallet_01", {0.02, 0, 0}, {0, 0, 0.05})
    .has_value());

  registry.Erase("pallet_01");
  EXPECT_EQ(registry.Find("pallet_01"), nullptr);
}

TEST(PalletModelFactory, BuildsPalletAndPayloadModelsFromTemplates)
{
  PalletModelFactory factory(PALLET_TEMPLATE_DIR);
  const gz::math::Pose3d pose(-2.5, -2.8, 0.0, 0.0, 0.0, 0.3);

  const auto pallet = factory.PalletModel("pallet_07", pose);
  EXPECT_EQ(pallet.Name(), "pallet_07");
  EXPECT_EQ(pallet.RawPose(), pose);
  const auto pallet_sdf = pallet.Element()->ToString("");
  EXPECT_NE(pallet_sdf.find("/warehouse/pallet/pallet_07/attach"), std::string::npos);
  EXPECT_NE(pallet_sdf.find("/warehouse/pallet/pallet_07/detach"), std::string::npos);
  EXPECT_EQ(pallet_sdf.find('@'), std::string::npos);

  const auto fresh = factory.PayloadModel("pallet_07", PalletKind::Fresh, pose);
  EXPECT_EQ(fresh.Name(), "pallet_07_payload");
  EXPECT_EQ(fresh.RawPose(), pose * gz::math::Pose3d(0, 0, 0.030, 0, 0, 0));
  const auto freshColor = fresh.LinkByName("payload_link")->VisualByIndex(0)->Material()->Diffuse();
  EXPECT_NEAR(freshColor.R(), 0.1, 1e-6);
  EXPECT_NEAR(freshColor.G(), 0.7, 1e-6);
  EXPECT_NEAR(freshColor.B(), 0.2, 1e-6);

  const auto normal = factory.PayloadModel("pallet_07", PalletKind::Normal, pose);
  EXPECT_EQ(normal.Name(), "pallet_07_payload");
  EXPECT_EQ(normal.RawPose(), pose * gz::math::Pose3d(0, 0, 0.030, 0, 0, 0));
  const auto normalColor = normal.LinkByName("payload_link")->VisualByIndex(0)->Material()->Diffuse();
  EXPECT_NEAR(normalColor.R(), 0.1, 1e-6);
  EXPECT_NEAR(normalColor.G(), 0.3, 1e-6);
  EXPECT_NEAR(normalColor.B(), 0.8, 1e-6);
}

TEST(PalletModelFactory, ReportsInvalidTemplate)
{
  PalletModelFactory factory(
    std::filesystem::path(PALLET_TEMPLATE_DIR) / "missing");
  try {
    (void)factory.PalletModel("pallet_07", gz::math::Pose3d::Zero);
    FAIL() << "missing template must throw";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(
      std::string(error.what()).find("MODEL_TEMPLATE_INVALID"),
      std::string::npos);
  }
}
}  // namespace mentorpi_gz_sim
