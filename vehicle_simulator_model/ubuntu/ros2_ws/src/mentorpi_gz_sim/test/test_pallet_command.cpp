#include <gtest/gtest.h>

#include "mentorpi_gz_sim/pallet_command.hpp"

namespace mentorpi_gz_sim
{
TEST(PalletCommand, ParsesSpawn)
{
  const auto result = ParseCommand(
    "spawn|pallet_07|fresh|loaded|-2.5|-2.8|0.0");

  ASSERT_TRUE(result.command.has_value());
  EXPECT_FALSE(result.error.has_value());
  EXPECT_EQ(result.command->type, CommandType::Spawn);
  EXPECT_EQ(result.command->id, "pallet_07");
  EXPECT_EQ(result.command->kind, PalletKind::Fresh);
  EXPECT_EQ(result.command->state, RequestedState::Loaded);
  ASSERT_TRUE(result.command->pose.has_value());
  EXPECT_DOUBLE_EQ(result.command->pose->Pos().X(), -2.5);
  EXPECT_DOUBLE_EQ(result.command->pose->Pos().Y(), -2.8);
  EXPECT_DOUBLE_EQ(result.command->pose->Rot().Yaw(), 0.0);
}

TEST(PalletCommand, RejectsInvalidIdAndPartialNumbers)
{
  const auto invalid_id = ParseCommand("remove|Pallet-1");
  ASSERT_TRUE(invalid_id.error.has_value());
  EXPECT_EQ(invalid_id.error->code, "INVALID_ID");

  const auto partial_number = ParseCommand(
    "spawn|pallet_1|fresh|loaded|1x|2|0");
  ASSERT_TRUE(partial_number.error.has_value());
  EXPECT_EQ(partial_number.error->code, "INVALID_NUMBER");
}

TEST(PalletCommand, ParsesStateRemoveAndList)
{
  const auto state = ParseCommand("state|pallet_1|empty|normal");
  ASSERT_TRUE(state.command.has_value());
  EXPECT_EQ(state.command->type, CommandType::State);
  EXPECT_EQ(state.command->id, "pallet_1");
  EXPECT_EQ(state.command->state, RequestedState::Empty);
  EXPECT_EQ(state.command->kind, PalletKind::Normal);
  EXPECT_FALSE(state.command->pose.has_value());

  const auto remove = ParseCommand("remove|pallet_1");
  ASSERT_TRUE(remove.command.has_value());
  EXPECT_EQ(remove.command->type, CommandType::Remove);
  EXPECT_EQ(remove.command->id, "pallet_1");
  EXPECT_FALSE(remove.command->kind.has_value());
  EXPECT_FALSE(remove.command->state.has_value());

  const auto list = ParseCommand("list");
  ASSERT_TRUE(list.command.has_value());
  EXPECT_EQ(list.command->type, CommandType::List);
  EXPECT_TRUE(list.command->id.empty());
}

TEST(PalletCommand, RejectsMalformedCommands)
{
  const auto invalid_field_count = ParseCommand("list|extra");
  ASSERT_TRUE(invalid_field_count.error.has_value());
  EXPECT_EQ(invalid_field_count.error->code, "INVALID_FIELD_COUNT");

  const auto invalid_kind = ParseCommand(
    "spawn|pallet_1|frozen|loaded|1|2|0");
  ASSERT_TRUE(invalid_kind.error.has_value());
  EXPECT_EQ(invalid_kind.error->code, "INVALID_KIND");

  const auto invalid_state = ParseCommand("state|pallet_1|full|fresh");
  ASSERT_TRUE(invalid_state.error.has_value());
  EXPECT_EQ(invalid_state.error->code, "INVALID_STATE");

  const auto non_finite = ParseCommand(
    "spawn|pallet_1|fresh|loaded|nan|2|0");
  ASSERT_TRUE(non_finite.error.has_value());
  EXPECT_EQ(non_finite.error->code, "INVALID_NUMBER");

  const auto unknown_command = ParseCommand("update|pallet_1");
  ASSERT_TRUE(unknown_command.error.has_value());
  EXPECT_EQ(unknown_command.error->code, "UNKNOWN_COMMAND");
}

TEST(PalletCommand, FormatsResponses)
{
  EXPECT_EQ(FormatOk("spawn", "pallet_1"), "ok|spawn|pallet_1");
  EXPECT_EQ(
    FormatError("INVALID_ID", "pallet id is invalid"),
    "error|INVALID_ID|pallet id is invalid");
}
}  // namespace mentorpi_gz_sim
