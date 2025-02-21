#ifndef CUSTOM_DIFF_DRIVE_HH_
#define CUSTOM_DIFF_DRIVE_HH_

#include <memory>
#include <string>
#include <mutex>

#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{
  /// \brief Custom differential drive plugin that subscribes to RPM values.
  class CustomDiffDrive
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor.
    public: CustomDiffDrive();

    /// \brief Destructor.
    public: ~CustomDiffDrive() override = default;

    /// \brief Called once when the plugin is loaded.
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    /// \brief Called every simulation iteration before physics is updated.
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    private: std::unique_ptr<class CustomDiffDrivePrivate> dataPtr;
  };
}
}
}
}

#endif // CUSTOM_DIFF_DRIVE_HH_
