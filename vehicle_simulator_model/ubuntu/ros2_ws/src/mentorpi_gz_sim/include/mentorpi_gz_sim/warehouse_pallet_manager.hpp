#ifndef MENTORPI_GZ_SIM__WAREHOUSE_PALLET_MANAGER_HPP_
#define MENTORPI_GZ_SIM__WAREHOUSE_PALLET_MANAGER_HPP_

#include <memory>

#include <gz/msgs/stringmsg.pb.h>
#include <gz/sim/System.hh>

namespace mentorpi_gz_sim
{
class WarehousePalletManagerPrivate;

class WarehousePalletManager final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
 public:
  WarehousePalletManager();
  ~WarehousePalletManager() override;

  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager & events) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override;

 private:
  bool OnCommand(
    const gz::msgs::StringMsg & request,
    gz::msgs::StringMsg & response);

  std::unique_ptr<WarehousePalletManagerPrivate> data_;
};
}  // namespace mentorpi_gz_sim

#endif  // MENTORPI_GZ_SIM__WAREHOUSE_PALLET_MANAGER_HPP_
