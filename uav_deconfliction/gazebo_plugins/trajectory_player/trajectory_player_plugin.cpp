#include "trajectory_player_plugin.h"

#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/Material.hh>
#include <gz/math/Color.hh>
#include <gz/plugin/Register.hh>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

using namespace gazebo;

class gazebo::TrajectoryPlayerPrivate
{
  public:
    // Model entity
    gz::sim::Entity modelEntity;
    
    // Model name
    std::string modelName;

    // File path
    std::string positionFilePath;
    
    // File stream
    std::ifstream positionFile;
    
    // Target
    gz::math::Vector3d targetPosition;
    gz::math::Quaterniond targetOrientation;
    gz::math::Color targetColor = gz::math::Color::White;
    
    // Cached Entities
    gz::sim::Entity visualEntity = gz::sim::kNullEntity;
    
    bool validTarget = false;
    double updateRateHz = 30.0;
    std::chrono::steady_clock::duration lastUpdateSimTime{0};
};

TrajectoryPlayer::TrajectoryPlayer()
  : dataPtr(std::make_unique<TrajectoryPlayerPrivate>())
{
}

TrajectoryPlayer::~TrajectoryPlayer() = default;

void TrajectoryPlayer::Configure(const gz::sim::Entity &_entity,
                                 const std::shared_ptr<const sdf::Element> &_sdf,
                                 gz::sim::EntityComponentManager &_ecm,
                                 gz::sim::EventManager &/*_eventMgr*/)
{
    this->dataPtr->modelEntity = _entity;
    
    // Get model name 
    // Usually the entity attached to is the model
    auto nameFn = _ecm.Component<gz::sim::components::Name>(_entity);
    if (nameFn)
       this->dataPtr->modelName = nameFn->Data();
    else
       this->dataPtr->modelName = "unknown_drone";

    if (_sdf->HasElement("update_rate"))
        this->dataPtr->updateRateHz = _sdf->Get<double>("update_rate");

    if (_sdf->HasElement("position_file"))
    {
        this->dataPtr->positionFilePath = _sdf->Get<std::string>("position_file");
    }
    else
    {
        this->dataPtr->positionFilePath = "/tmp/gazebo_drone_positions/" + this->dataPtr->modelName + ".txt";
    }

    std::cout << "[TrajectoryPlayer] Configured for model: " << this->dataPtr->modelName << std::endl;
}

void TrajectoryPlayer::PreUpdate(const gz::sim::UpdateInfo &_info,
                                 gz::sim::EntityComponentManager &_ecm)
{
    // Check update rate
    // _info.simTime is std::chrono::steady_clock::duration
    double dt = 1.0 / this->dataPtr->updateRateHz;
    auto dtDur = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(dt));
    
    if (_info.simTime - this->dataPtr->lastUpdateSimTime < dtDur)
        return;

    this->dataPtr->lastUpdateSimTime = _info.simTime;
    
    // File Read Logic (Same as before)
    if (!this->dataPtr->positionFile.is_open())
    {
        this->dataPtr->positionFile.open(this->dataPtr->positionFilePath);
    }
    
    if (this->dataPtr->positionFile.is_open())
    {
        std::string line, lastLine;
        while (std::getline(this->dataPtr->positionFile, line))
        {
            if (!line.empty()) lastLine = line;
        }

        if (!lastLine.empty())
        {
            std::istringstream iss(lastLine);
            std::string token;
            std::vector<double> values;
            while (std::getline(iss, token, ','))
            {
                try { values.push_back(std::stod(token)); } catch(...) {}
            }
            
            if (values.size() >= 4)
            {
                this->dataPtr->targetPosition.Set(values[1], values[2], values[3]);
                
                if (values.size() >= 7)
                    this->dataPtr->targetOrientation = gz::math::Quaterniond(values[4], values[5], values[6]);
                else
                    this->dataPtr->targetOrientation = gz::math::Quaterniond::Identity;

                if (values.size() >= 10)
                     this->dataPtr->targetColor.Set(values[7], values[8], values[9], 1.0);

                this->dataPtr->validTarget = true;
            }
        }
        
        this->dataPtr->positionFile.clear();
        this->dataPtr->positionFile.seekg(0, std::ios::beg);
    }

// Update Pose if valid
    if (this->dataPtr->validTarget)
    {
        // 1. Update Pose
        gz::math::Pose3d pose(this->dataPtr->targetPosition, this->dataPtr->targetOrientation);
        auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->dataPtr->modelEntity);
        if (poseComp)
        {
            *poseComp = gz::sim::components::Pose(pose);
            _ecm.SetChanged(this->dataPtr->modelEntity, gz::sim::components::Pose::typeId, gz::sim::ComponentState::OneTimeChange);
        }
        else
        {
            _ecm.CreateComponent(this->dataPtr->modelEntity, gz::sim::components::Pose(pose));
        }

        // 2. Update Color
        // Only run if we have a valid color parsed (values size >= 10) or we want defaults
        // Logic to find visual entity (lazy init)
        if (this->dataPtr->visualEntity == gz::sim::kNullEntity) {
            // Traverse Model -> Link -> Visual
            // This is a simplified traversal assuming first link/first visual
            auto links = _ecm.ChildrenByComponents(this->dataPtr->modelEntity, gz::sim::components::Link());
            if (!links.empty()) {
                auto linkEntity = links[0];
                auto visuals = _ecm.ChildrenByComponents(linkEntity, gz::sim::components::Visual());
                if (!visuals.empty()) {
                    this->dataPtr->visualEntity = visuals[0];
                    // std::cout << "Found visual entity: " << this->dataPtr->visualEntity << std::endl;
                }
            }
        }

        if (this->dataPtr->visualEntity != gz::sim::kNullEntity) {
            // Parse RBG from our simplified storage or re-parse?
            // We need to store color in private data first.
            // Let's assume we parsed it into this->dataPtr->targetColor (Vector3d)
            
            // Check if Material component exists
            auto matComp = _ecm.Component<gz::sim::components::Material>(this->dataPtr->visualEntity);
            if (matComp) {
                // Get current ambient/diffuse to compare? Or just overwrite.
                auto &mat = matComp->Data();
                
                // Update Diffuse
                mat.SetDiffuse(this->dataPtr->targetColor);
                mat.SetAmbient(this->dataPtr->targetColor); // Set ambient too for visibility
                
                // Mark changed
                _ecm.SetChanged(this->dataPtr->visualEntity, gz::sim::components::Material::typeId, gz::sim::ComponentState::OneTimeChange);
            } else {
                 // Create if missing?
                 sdf::Material newMat;
                 newMat.SetDiffuse(this->dataPtr->targetColor);
                 newMat.SetAmbient(this->dataPtr->targetColor);
                 _ecm.CreateComponent(this->dataPtr->visualEntity, gz::sim::components::Material(newMat));
            }
        }
    }
}

GZ_ADD_PLUGIN(
    gazebo::TrajectoryPlayer,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate
)

// Add alias for SDF use
GZ_ADD_PLUGIN_ALIAS(gazebo::TrajectoryPlayer, "gazebo::TrajectoryPlayer")
