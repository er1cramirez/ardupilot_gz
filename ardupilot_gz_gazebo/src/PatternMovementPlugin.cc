#include <cmath>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/math/Pose3.hh>
#include <gz/common/Console.hh>

#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{

/// \brief Pattern movement plugin for Gazebo models
/// This plugin applies predefined movement patterns to a model.
/// Supported patterns: line, circle, infinity
class PatternMovementPlugin
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate
{
  /// \brief Constructor
  public: PatternMovementPlugin() = default;

  /// \brief Destructor
  public: ~PatternMovementPlugin() override = default;

  /// \brief Configure the plugin
  /// \param[in] _entity The entity the plugin is attached to
  /// \param[in] _sdf The SDF element containing plugin configuration
  /// \param[in] _ecm The entity component manager
  /// \param[in] _eventMgr The event manager
  public: void Configure(const Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              EntityComponentManager &_ecm,
              EventManager &_eventMgr) override
  {
    this->model = Model(_entity);
    if (!this->model.Valid(_ecm)) {
      gzerr << "PatternMovementPlugin attached to invalid model entity ["
            << _entity << "]" << std::endl;
      return;
    }

    // Get model name
    this->modelName = this->model.Name(_ecm);
    gzmsg << "PatternMovementPlugin configured for model ["
          << this->modelName << "]" << std::endl;

    // Get the movement pattern type
    if (_sdf->HasElement("pattern")) {
      this->pattern = _sdf->Get<std::string>("pattern");
      gzmsg << "Using movement pattern: " << this->pattern << std::endl;
    } else {
      this->pattern = "line";
      gzmsg << "No pattern specified, using default: line" << std::endl;
    }

    // Get the linear velocity
    if (_sdf->HasElement("linear_velocity")) {
      this->linearVel = _sdf->Get<double>("linear_velocity");
    } else {
      this->linearVel = 0.5;  // Default: 0.5 m/s
    }
    gzmsg << "Linear velocity: " << this->linearVel << " m/s" << std::endl;

    // Get the radius (for circle and infinity patterns)
    if (_sdf->HasElement("radius")) {
      this->radius = _sdf->Get<double>("radius");
    } else {
      this->radius = 1.0;  // Default: 1.0 meter
    }
    gzmsg << "Path radius: " << this->radius << " m" << std::endl;

    // Get the initial pose
    auto pose = _ecm.Component<components::Pose>(this->model.Entity());
    if (pose) {
      this->initialPose = pose->Data();
      gzmsg << "Initial pose: " << this->initialPose << std::endl;
    }

    this->startTime = -1.0;  // Will be initialized on first update
  }

  /// \brief Update the model pose based on the selected pattern
  /// \param[in] _info Update information
  /// \param[in] _ecm The entity component manager
  public: void PreUpdate(const UpdateInfo &_info,
              EntityComponentManager &_ecm) override
  {
    // Skip if paused
    if (_info.paused) {
      return;
    }

    // Get the current simulation time
    const double currentTime = _info.simTime.count() / 1e9;
    
    // Initialize start time on first update
    if (this->startTime < 0) {
      this->startTime = currentTime;
      return;
    }

    // Calculate elapsed time since start
    double elapsedTime = currentTime - this->startTime;
    
    // Calculate new pose based on pattern
    math::Pose3d newPose = this->initialPose;
    double angularSpeed = this->linearVel / this->radius;  // ω = v/r for circular motion
    
    if (this->pattern == "line") {
      // Simple linear movement along x-axis
      double distance = this->linearVel * elapsedTime;
      newPose.Pos().X() = this->initialPose.Pos().X() + distance;
    }
    else if (this->pattern == "circle") {
      // Circular movement in the xy plane
      double angle = angularSpeed * elapsedTime;
      newPose.Pos().X() = this->initialPose.Pos().X() + this->radius * cos(angle);
      newPose.Pos().Y() = this->initialPose.Pos().Y() + this->radius * sin(angle);
      
      // Make the model face the direction of movement
      newPose.Rot().Euler(0, 0, angle + M_PI/2);
    }
    else if (this->pattern == "infinity") {
      // Figure-8 pattern (lemniscate of Bernoulli)
      double t = angularSpeed * elapsedTime;
      double denominator = 1.0 + std::pow(sin(t), 2);
      
      newPose.Pos().X() = this->initialPose.Pos().X() + this->radius * cos(t) / denominator;
      newPose.Pos().Y() = this->initialPose.Pos().Y() + this->radius * sin(t) * cos(t) / denominator;
      
      // Calculate tangent angle for rotation
      double tangentAngle = atan2(cos(t) * cos(2*t), -sin(t));
      newPose.Rot().Euler(0, 0, tangentAngle);
    }
    
    // Update the model's pose
    auto poseComp = _ecm.Component<components::Pose>(this->model.Entity());
    if (poseComp) {
      _ecm.SetComponentData<components::Pose>(this->model.Entity(), newPose);
    }
  }

  private: Model model{kNullEntity};
  private: std::string modelName;
  private: std::string pattern{"line"};
  private: double linearVel{0.5};  // m/s
  private: double radius{1.0};     // meters
  private: math::Pose3d initialPose;
  private: double startTime{-1.0};
};

GZ_ADD_PLUGIN(PatternMovementPlugin,
              System,
              PatternMovementPlugin::ISystemConfigure,
              PatternMovementPlugin::ISystemPreUpdate)

}  // namespace sim
}  // namespace gz