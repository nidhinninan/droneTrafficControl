#ifndef TRAJECTORY_PLAYER_SYSTEM_H
#define TRAJECTORY_PLAYER_SYSTEM_H

#include <gz/sim/System.hh>
#include <memory> 

namespace gazebo
{
  // Forward declare private data class
  class TrajectoryPlayerPrivate;

  // This plugin system controls a model's pose based on file input
  class TrajectoryPlayer : public gz::sim::System,
                           public gz::sim::ISystemConfigure,
                           public gz::sim::ISystemPreUpdate
  {
    public:
      TrajectoryPlayer();
      ~TrajectoryPlayer() override;

      void Configure(const gz::sim::Entity &_entity,
                     const std::shared_ptr<const sdf::Element> &_sdf,
                     gz::sim::EntityComponentManager &_ecm,
                     gz::sim::EventManager &_eventMgr) override;

      void PreUpdate(const gz::sim::UpdateInfo &_info,
                     gz::sim::EntityComponentManager &_ecm) override;

    private:
      std::unique_ptr<TrajectoryPlayerPrivate> dataPtr;
  };
}

#endif
