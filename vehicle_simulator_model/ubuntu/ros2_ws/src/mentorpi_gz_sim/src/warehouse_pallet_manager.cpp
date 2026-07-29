#include "mentorpi_gz_sim/warehouse_pallet_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/empty.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include "mentorpi_gz_sim/pallet_command.hpp"
#include "mentorpi_gz_sim/pallet_model_factory.hpp"
#include "mentorpi_gz_sim/pallet_registry.hpp"

namespace mentorpi_gz_sim
{
namespace
{
constexpr std::chrono::seconds kCommandWait{5};
constexpr std::chrono::milliseconds kOperationTimeout{4500};

enum class ActiveStage
{
  Start,
  WaitForJoint,
  WaitForJointRemoval,
  WaitForPayloadRemoval,
  WaitForPalletRemoval
};

enum class AfterPayloadRemoval
{
  StateEmpty,
  StartReplacement,
  RemovePallet
};

struct PendingCommand
{
  Command command;
  bool internal{false};
  std::promise<std::string> result;
};

struct ActiveCommand
{
  std::shared_ptr<PendingCommand> pending;
  ActiveStage stage{ActiveStage::Start};
  AfterPayloadRemoval afterPayloadRemoval{AfterPayloadRemoval::StateEmpty};
  std::optional<PalletKind> targetKind;
  std::chrono::steady_clock::time_point started{std::chrono::steady_clock::now()};
};

std::string CommandName(CommandType type)
{
  switch (type) {
    case CommandType::Spawn:
      return "spawn";
    case CommandType::State:
      return "state";
    case CommandType::Remove:
      return "remove";
    case CommandType::List:
      return "list";
  }
  return "unknown";
}

std::string KindName(PalletKind kind)
{
  return kind == PalletKind::Fresh ? "fresh" : "normal";
}

std::string StateName(StableState state)
{
  return state == StableState::Loaded ? "loaded" : "empty";
}

std::optional<PalletKind> KindFromString(const std::string & value)
{
  if (value == "fresh") {
    return PalletKind::Fresh;
  }
  if (value == "normal") {
    return PalletKind::Normal;
  }
  return std::nullopt;
}

std::optional<RequestedState> StateFromString(const std::string & value)
{
  if (value == "empty") {
    return RequestedState::Empty;
  }
  if (value == "loaded") {
    return RequestedState::Loaded;
  }
  return std::nullopt;
}

std::optional<gz::math::Pose3d> ParsePose(const std::string & value)
{
  std::istringstream input(value);
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
  if (!(input >> x >> y >> z >> roll >> pitch >> yaw)) {
    return std::nullopt;
  }
  input >> std::ws;
  if (!input.eof()) {
    return std::nullopt;
  }
  return gz::math::Pose3d{x, y, z, roll, pitch, yaw};
}

gz::math::Vector4d ParseBounds(
  const std::string & value,
  const gz::math::Vector4d & fallback)
{
  std::istringstream input(value);
  double minX;
  double maxX;
  double minY;
  double maxY;
  if (!(input >> minX >> maxX >> minY >> maxY)) {
    return fallback;
  }
  input >> std::ws;
  if (!input.eof()) {
    return fallback;
  }
  return {minX, maxX, minY, maxY};
}

std::filesystem::path ResolveTemplateDirectory(const std::string & configured)
{
  const std::filesystem::path path{configured};
  if (path.is_absolute() || std::filesystem::exists(path)) {
    return path;
  }

  const char * resourcePath = std::getenv("GZ_SIM_RESOURCE_PATH");
  if (resourcePath == nullptr) {
    return path;
  }

  std::istringstream paths(resourcePath);
  std::string base;
  std::filesystem::path firstCandidate;
  while (std::getline(paths, base, ':')) {
    if (base.empty()) {
      continue;
    }
    const auto candidate = std::filesystem::path(base) / path;
    if (firstCandidate.empty()) {
      firstCandidate = candidate;
    }
    if (std::filesystem::exists(candidate)) {
      return candidate;
    }
  }
  return firstCandidate.empty() ? path : firstCandidate;
}

std::string Attribute(
  const sdf::ElementPtr & element,
  const std::string & name)
{
  const auto parameter = element->GetAttribute(name);
  return parameter ? parameter->GetAsString() : std::string{};
}
}  // namespace

class WarehousePalletManagerPrivate
{
 public:
  explicit WarehousePalletManagerPrivate(WarehousePalletManager * owner)
  : owner(owner), registry({-4.5, 4.5, -3.5, 3.5}, 0.20)
  {
  }

  void Queue(std::shared_ptr<PendingCommand> pending)
  {
    std::lock_guard<std::mutex> lock(queueMutex);
    queue.push_back(std::move(pending));
  }

  std::shared_ptr<PendingCommand> Pop()
  {
    std::lock_guard<std::mutex> lock(queueMutex);
    if (queue.empty()) {
      return nullptr;
    }
    auto pending = queue.front();
    queue.pop_front();
    return pending;
  }

  void Finish(std::string response)
  {
    if (active->pending->internal) {
      if (response.rfind("error|", 0) == 0) {
        gzerr << "Default pallet command failed: " << response << std::endl;
      }
    } else {
      active->pending->result.set_value(std::move(response));
    }
    active.reset();
  }

  void FinishOk()
  {
    const auto & command = active->pending->command;
    Finish(FormatOk(CommandName(command.type), command.id));
  }

  void FinishError(const std::string & code)
  {
    Finish(FormatError(code, active->pending->command.id));
  }

  bool HasJoint(
    const PalletRecord & record,
    gz::sim::EntityComponentManager & ecm) const
  {
    bool found = false;
    ecm.Each<gz::sim::components::DetachableJoint>(
      [&found, &record](
        const gz::sim::Entity &,
        const gz::sim::components::DetachableJoint * joint)
      {
        const auto & info = joint->Data();
        found = info.parentLink == record.palletLink &&
          info.childLink == record.payloadLink;
        return !found;
      });
    return found;
  }

  bool PublishEmpty(const std::string & topic)
  {
    auto publisher = node.Advertise<gz::msgs::Empty>(topic);
    if (!publisher) {
      return false;
    }
    gz::msgs::Empty message;
    return publisher.Publish(message);
  }

  std::optional<CommandError> ValidateMutable(
    const std::string & id,
    gz::sim::EntityComponentManager & ecm)
  {
    gz::math::Vector3d linear = gz::math::Vector3d::Zero;
    gz::math::Vector3d angular = gz::math::Vector3d::Zero;
    const auto * record = registry.Find(id);
    if (record != nullptr && record->palletLink != gz::sim::kNullEntity &&
      ecm.HasEntity(record->palletLink))
    {
      const gz::sim::Link link(record->palletLink);
      linear = link.WorldLinearVelocity(ecm).value_or(gz::math::Vector3d::Zero);
      angular = link.WorldAngularVelocity(ecm).value_or(gz::math::Vector3d::Zero);
    }
    return registry.ValidateMutable(id, linear, angular);
  }

  void RefreshOccupiedPoses(gz::sim::EntityComponentManager & ecm)
  {
    std::vector<gz::math::Vector2d> occupied;
    occupied.reserve(ids.size() + 2);

    for (const auto & id : ids) {
      const auto * record = registry.Find(id);
      if (record == nullptr || record->palletEntity == gz::sim::kNullEntity ||
        !ecm.HasEntity(record->palletEntity))
      {
        continue;
      }
      const auto pose = gz::sim::worldPose(record->palletEntity, ecm);
      occupied.emplace_back(pose.Pos().X(), pose.Pos().Y());
    }

    for (const std::string robotName : {"robot_1", "robot_2"}) {
      const auto robot = ecm.EntityByComponents(
        gz::sim::components::Model(),
        gz::sim::components::Name(robotName));
      if (robot == gz::sim::kNullEntity) {
        continue;
      }
      const auto pose = gz::sim::worldPose(robot, ecm);
      occupied.emplace_back(pose.Pos().X(), pose.Pos().Y());
    }
    registry.SetOccupiedPoses(std::move(occupied));
  }

  bool CreatePayload(
    PalletRecord & record,
    PalletKind kind,
    gz::sim::EntityComponentManager & ecm)
  {
    try {
      const auto palletPose = gz::sim::worldPose(record.palletEntity, ecm);
      auto payload = factory->PayloadModel(record.id, kind, palletPose);
      record.payloadEntity = creator->CreateEntities(&payload);
      if (record.payloadEntity == gz::sim::kNullEntity) {
        return false;
      }
      creator->SetParent(record.payloadEntity, worldEntity);
      record.payloadLink =
        gz::sim::Model(record.payloadEntity).CanonicalLink(ecm);
      if (record.payloadLink == gz::sim::kNullEntity) {
        creator->RequestRemoveEntity(record.payloadEntity, true);
        record.payloadEntity = gz::sim::kNullEntity;
        return false;
      }
      if (!PublishEmpty("/warehouse/pallet/" + record.id + "/attach")) {
        return false;
      }
      return true;
    } catch (const std::runtime_error &) {
      throw;
    }
  }

  void EraseRecord(const std::string & id)
  {
    registry.Erase(id);
    ids.erase(std::remove(ids.begin(), ids.end(), id), ids.end());
  }

  void StartSpawn(gz::sim::EntityComponentManager & ecm)
  {
    const auto & command = active->pending->command;
    RefreshOccupiedPoses(ecm);
    if (const auto error = registry.ValidateSpawn(
        command.id, {command.pose->Pos().X(), command.pose->Pos().Y()}))
    {
      FinishError(error->code);
      return;
    }

    PalletRecord record;
    record.id = command.id;
    record.kind = *command.kind;
    try {
      auto pallet = factory->PalletModel(command.id, *command.pose);
      record.palletEntity = creator->CreateEntities(&pallet);
      if (record.palletEntity == gz::sim::kNullEntity) {
        FinishError("ENTITY_CREATION_FAILED");
        return;
      }
      creator->SetParent(record.palletEntity, worldEntity);
      record.palletLink =
        gz::sim::Model(record.palletEntity).CanonicalLink(ecm);
      if (record.palletLink == gz::sim::kNullEntity) {
        creator->RequestRemoveEntity(record.palletEntity, true);
        FinishError("ENTITY_CREATION_FAILED");
        return;
      }
      gz::sim::Link(record.palletLink).EnableVelocityChecks(ecm, true);
      registry.Insert(record);
      ids.push_back(command.id);

      auto * inserted = registry.Find(command.id);
      if (*command.state == RequestedState::Empty) {
        inserted->state = StableState::Empty;
        FinishOk();
        return;
      }

      inserted->transition = TransitionState::Loading;
      active->targetKind = command.kind;
      if (!CreatePayload(*inserted, *command.kind, ecm)) {
        creator->RequestRemoveEntity(inserted->palletEntity, true);
        EraseRecord(command.id);
        FinishError("ENTITY_CREATION_FAILED");
        return;
      }
      active->stage = ActiveStage::WaitForJoint;
    } catch (const std::runtime_error &) {
      if (const auto * inserted = registry.Find(command.id)) {
        creator->RequestRemoveEntity(inserted->payloadEntity, true);
        creator->RequestRemoveEntity(inserted->palletEntity, true);
        EraseRecord(command.id);
      }
      FinishError("MODEL_TEMPLATE_INVALID");
    }
  }

  void StartState(gz::sim::EntityComponentManager & ecm)
  {
    const auto & command = active->pending->command;
    if (const auto error = ValidateMutable(command.id, ecm)) {
      FinishError(error->code);
      return;
    }

    auto * record = registry.Find(command.id);
    if (*command.state == RequestedState::Empty) {
      if (record->state == StableState::Empty) {
        FinishOk();
        return;
      }
      record->transition = TransitionState::Unloading;
      active->afterPayloadRemoval = AfterPayloadRemoval::StateEmpty;
      if (!PublishEmpty("/warehouse/pallet/" + command.id + "/detach")) {
        FinishError("TRANSPORT_PUBLISH_FAILED");
        return;
      }
      active->stage = ActiveStage::WaitForJointRemoval;
      return;
    }

    active->targetKind = command.kind;
    if (record->state == StableState::Empty) {
      record->transition = TransitionState::Loading;
      try {
        if (!CreatePayload(*record, *command.kind, ecm)) {
          FinishError("ENTITY_CREATION_FAILED");
          return;
        }
        active->stage = ActiveStage::WaitForJoint;
      } catch (const std::runtime_error &) {
        FinishError("MODEL_TEMPLATE_INVALID");
      }
      return;
    }

    if (record->kind == *command.kind) {
      FinishOk();
      return;
    }

    record->transition = TransitionState::Replacing;
    active->afterPayloadRemoval = AfterPayloadRemoval::StartReplacement;
    if (!PublishEmpty("/warehouse/pallet/" + command.id + "/detach")) {
      FinishError("TRANSPORT_PUBLISH_FAILED");
      return;
    }
    active->stage = ActiveStage::WaitForJointRemoval;
  }

  void StartRemove(gz::sim::EntityComponentManager & ecm)
  {
    const auto & command = active->pending->command;
    if (const auto error = ValidateMutable(command.id, ecm)) {
      FinishError(error->code);
      return;
    }

    auto * record = registry.Find(command.id);
    if (record->state == StableState::Loaded) {
      record->transition = TransitionState::Unloading;
      active->afterPayloadRemoval = AfterPayloadRemoval::RemovePallet;
      if (!PublishEmpty("/warehouse/pallet/" + command.id + "/detach")) {
        FinishError("TRANSPORT_PUBLISH_FAILED");
        return;
      }
      active->stage = ActiveStage::WaitForJointRemoval;
      return;
    }

    creator->RequestRemoveEntity(record->palletEntity, true);
    active->stage = ActiveStage::WaitForPalletRemoval;
  }

  void StartList()
  {
    std::vector<std::string> sorted = ids;
    std::sort(sorted.begin(), sorted.end());
    std::ostringstream details;
    bool first = true;
    for (const auto & id : sorted) {
      const auto * record = registry.Find(id);
      if (record == nullptr) {
        continue;
      }
      if (!first) {
        details << ',';
      }
      first = false;
      details << id << ':' << KindName(record->kind) << ':'
              << StateName(record->state);
    }
    Finish(FormatOk("list", details.str()));
  }

  void ProcessStart(gz::sim::EntityComponentManager & ecm)
  {
    switch (active->pending->command.type) {
      case CommandType::Spawn:
        StartSpawn(ecm);
        break;
      case CommandType::State:
        StartState(ecm);
        break;
      case CommandType::Remove:
        StartRemove(ecm);
        break;
      case CommandType::List:
        StartList();
        break;
    }
  }

  void ProcessWaitForJoint(gz::sim::EntityComponentManager & ecm)
  {
    auto * record = registry.Find(active->pending->command.id);
    if (record == nullptr || !HasJoint(*record, ecm)) {
      return;
    }
    record->kind = *active->targetKind;
    record->state = StableState::Loaded;
    record->transition = TransitionState::None;
    FinishOk();
  }

  void ProcessWaitForJointRemoval(gz::sim::EntityComponentManager & ecm)
  {
    auto * record = registry.Find(active->pending->command.id);
    if (record == nullptr || HasJoint(*record, ecm)) {
      return;
    }
    if (record->payloadEntity != gz::sim::kNullEntity &&
      ecm.HasEntity(record->payloadEntity))
    {
      creator->RequestRemoveEntity(record->payloadEntity, true);
    }
    active->stage = ActiveStage::WaitForPayloadRemoval;
  }

  void ProcessWaitForPayloadRemoval(gz::sim::EntityComponentManager & ecm)
  {
    auto * record = registry.Find(active->pending->command.id);
    if (record == nullptr) {
      FinishError("NOT_FOUND");
      return;
    }
    if (record->payloadEntity != gz::sim::kNullEntity &&
      ecm.HasEntity(record->payloadEntity))
    {
      return;
    }

    record->payloadEntity = gz::sim::kNullEntity;
    record->payloadLink = gz::sim::kNullEntity;
    record->state = StableState::Empty;

    if (active->afterPayloadRemoval == AfterPayloadRemoval::StateEmpty) {
      record->transition = TransitionState::None;
      FinishOk();
      return;
    }
    if (active->afterPayloadRemoval == AfterPayloadRemoval::RemovePallet) {
      creator->RequestRemoveEntity(record->palletEntity, true);
      active->stage = ActiveStage::WaitForPalletRemoval;
      return;
    }

    record->transition = TransitionState::Loading;
    try {
      if (!CreatePayload(*record, *active->targetKind, ecm)) {
        FinishError("ENTITY_CREATION_FAILED");
        return;
      }
      active->stage = ActiveStage::WaitForJoint;
    } catch (const std::runtime_error &) {
      FinishError("MODEL_TEMPLATE_INVALID");
    }
  }

  void ProcessWaitForPalletRemoval(gz::sim::EntityComponentManager & ecm)
  {
    const auto id = active->pending->command.id;
    const auto * record = registry.Find(id);
    if (record != nullptr && record->palletEntity != gz::sim::kNullEntity &&
      ecm.HasEntity(record->palletEntity))
    {
      return;
    }
    EraseRecord(id);
    FinishOk();
  }

  void Process(gz::sim::EntityComponentManager & ecm)
  {
    if (!active) {
      if (auto next = Pop()) {
        active.emplace(ActiveCommand{std::move(next)});
      } else {
        return;
      }
    }

    if (std::chrono::steady_clock::now() - active->started >
      kOperationTimeout)
    {
      FinishError("OPERATION_TIMEOUT");
      return;
    }

    switch (active->stage) {
      case ActiveStage::Start:
        ProcessStart(ecm);
        break;
      case ActiveStage::WaitForJoint:
        ProcessWaitForJoint(ecm);
        break;
      case ActiveStage::WaitForJointRemoval:
        ProcessWaitForJointRemoval(ecm);
        break;
      case ActiveStage::WaitForPayloadRemoval:
        ProcessWaitForPayloadRemoval(ecm);
        break;
      case ActiveStage::WaitForPalletRemoval:
        ProcessWaitForPalletRemoval(ecm);
        break;
    }
  }

  WarehousePalletManager * owner;
  gz::transport::Node node;
  gz::sim::Entity worldEntity{gz::sim::kNullEntity};
  gz::sim::EventManager * events{nullptr};
  std::unique_ptr<gz::sim::SdfEntityCreator> creator;
  std::unique_ptr<PalletModelFactory> factory;
  PalletRegistry registry;
  std::string commandService{"/warehouse/pallet/command"};
  std::mutex queueMutex;
  std::deque<std::shared_ptr<PendingCommand>> queue;
  std::optional<ActiveCommand> active;
  std::vector<std::string> ids;
};

WarehousePalletManager::WarehousePalletManager()
: data_(std::make_unique<WarehousePalletManagerPrivate>(this))
{
}

WarehousePalletManager::~WarehousePalletManager() = default;

void WarehousePalletManager::Configure(
  const gz::sim::Entity & entity,
  const std::shared_ptr<const sdf::Element> & sdf,
  gz::sim::EntityComponentManager & ecm,
  gz::sim::EventManager & events)
{
  data_->worldEntity = entity;
  data_->events = &events;
  data_->creator = std::make_unique<gz::sim::SdfEntityCreator>(ecm, events);

  data_->commandService = sdf->Get<std::string>(
    "command_service", data_->commandService).first;
  const auto configuredTemplateDir = sdf->Get<std::string>(
    "template_dir", "pallet").first;
  data_->factory = std::make_unique<PalletModelFactory>(
    ResolveTemplateDirectory(configuredTemplateDir));

  const gz::math::Vector4d defaultBounds{-4.5, 4.5, -3.5, 3.5};
  const auto bounds = ParseBounds(
    sdf->Get<std::string>(
      "spawn_bounds", "-4.5 4.5 -3.5 3.5").first,
    defaultBounds);
  data_->registry = PalletRegistry(bounds, 0.20);

  if (sdf->HasElement("default_pallet")) {
    auto defaultPallet = sdf->GetFirstElement();
    while (defaultPallet != nullptr) {
      if (defaultPallet->GetName() != "default_pallet") {
        defaultPallet = defaultPallet->GetNextElement();
        continue;
      }
      const auto id = Attribute(defaultPallet, "id");
      const auto kind = KindFromString(Attribute(defaultPallet, "kind"));
      const auto state = StateFromString(Attribute(defaultPallet, "state"));
      const auto pose = ParsePose(Attribute(defaultPallet, "pose"));
      if (id.empty() || !kind || !state || !pose) {
        gzerr << "Ignoring invalid default_pallet configuration for ["
              << id << "]" << std::endl;
      } else {
        auto pending = std::make_shared<PendingCommand>();
        pending->command = Command{
          CommandType::Spawn, id, kind, state, pose};
        pending->internal = true;
        data_->Queue(std::move(pending));
      }
      defaultPallet = defaultPallet->GetNextElement();
    }
  }

  if (!data_->node.Advertise(
      data_->commandService, &WarehousePalletManager::OnCommand, this))
  {
    gzerr << "Failed to advertise pallet command service ["
          << data_->commandService << "]" << std::endl;
  }
}

void WarehousePalletManager::PreUpdate(
  const gz::sim::UpdateInfo &,
  gz::sim::EntityComponentManager & ecm)
{
  data_->Process(ecm);
}

bool WarehousePalletManager::OnCommand(
  const gz::msgs::StringMsg & request,
  gz::msgs::StringMsg & response)
{
  const auto parsed = ParseCommand(request.data());
  if (parsed.error) {
    response.set_data(FormatError(parsed.error->code, parsed.error->detail));
    return true;
  }

  auto pending = std::make_shared<PendingCommand>();
  pending->command = *parsed.command;
  auto future = pending->result.get_future();
  data_->Queue(std::move(pending));

  if (future.wait_for(kCommandWait) != std::future_status::ready) {
    response.set_data(FormatError(
      "COMMAND_TIMEOUT",
      parsed.command->id.empty() ? CommandName(parsed.command->type) :
      parsed.command->id));
    return true;
  }
  response.set_data(future.get());
  return true;
}
}  // namespace mentorpi_gz_sim

GZ_ADD_PLUGIN(
  mentorpi_gz_sim::WarehousePalletManager,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(
  mentorpi_gz_sim::WarehousePalletManager,
  "mentorpi_gz_sim::WarehousePalletManager")
