#include <cmath>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Pose3.hh>
#include <gz/common/Console.hh>
#include <gz/msgs/twist.pb.h>

#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{

class PatternMovementPlugin
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate
{
  public: PatternMovementPlugin() = default;
  public: ~PatternMovementPlugin() override = default;

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

    // Get the command topic
    if (_sdf->HasElement("topic")) {
      this->topic = _sdf->Get<std::string>("topic");
    } else {
      this->topic = "/model/" + this->modelName + "/cmd_vel";
    }
    gzmsg << "Command topic: " << this->topic << std::endl;

    // Initialize the transport node (no Init() needed in newer Gazebo versions)
    this->node = std::make_unique<transport::Node>();
    // Create the publisher
    this->cmdVelPub = this->node->Advertise<msgs::Twist>(this->topic);

    // Get the initial pose
    auto pose = _ecm.Component<components::Pose>(this->model.Entity());
    if (pose) {
      this->initialPose = pose->Data();
      gzmsg << "Initial pose: " << this->initialPose << std::endl;
    }
    else {
      gzerr << "Failed to get initial pose for model [" << this->modelName << "]" << std::endl;
    }

    this->startTime = -1.0;  // Will be initialized on first update
  }

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
      gzmsg << "PatternMovementPlugin: Initializing start time for model [" 
            << this->modelName << "] at " << currentTime << std::endl;
      this->startTime = currentTime;
      this->prevTime = currentTime;
      return;
    }

    // Only publish commands at a reasonable rate
    if (currentTime - this->prevTime < 0.05) {
      return;
    }
    this->prevTime = currentTime;

    // Calculate elapsed time since start
    double elapsedTime = currentTime - this->startTime;
    
    // Print debug messages periodically
    if (_info.iterations % 200 == 0) {
      gzmsg << "PatternMovementPlugin: Moving model [" << this->modelName 
            << "] at time " << elapsedTime << " seconds" << std::endl;
    }
    
    // Calculate velocity commands based on pattern
    msgs::Twist msg;
    
    if (this->pattern == "line") {
      // Simple linear movement along x-axis
      msg.mutable_linear()->set_x(this->linearVel);
      msg.mutable_linear()->set_y(0);
      msg.mutable_angular()->set_z(0);
    }
    else if (this->pattern == "circle") {
      // Circular movement requires both linear and angular velocity
      double angularVel = this->linearVel / this->radius;
      msg.mutable_linear()->set_x(this->linearVel);
      msg.mutable_linear()->set_y(0);
      msg.mutable_angular()->set_z(angularVel);
    }
    else if (this->pattern == "infinity") {
      // Figure-8 pattern requires varying linear and angular velocity
      double t = (this->linearVel / this->radius) * elapsedTime;
      
      // Calculate tangent angle change rate for the figure-8
      double angularVel = (this->linearVel / this->radius) * 
                          std::cos(2 * t) / (1 + std::pow(std::sin(t), 2));
      
      msg.mutable_linear()->set_x(this->linearVel);
      msg.mutable_linear()->set_y(0);
      msg.mutable_angular()->set_z(angularVel);
    }
    
    // Publish the command
    this->cmdVelPub.Publish(msg);
    
    if (_info.iterations % 200 == 0) {
      gzmsg << "PatternMovementPlugin: Published command for model [" 
            << this->modelName << "]: linear.x=" << msg.linear().x()
            << ", angular.z=" << msg.angular().z() << std::endl;
    }
  }

  private: Model model{kNullEntity};
  private: std::string modelName;
  private: std::string pattern{"line"};
  private: double linearVel{0.5};  // m/s
  private: double radius{1.0};     // meters
  private: math::Pose3d initialPose;
  private: double startTime{-1.0};
  private: double prevTime{0.0};
  private: std::string topic;
  private: std::unique_ptr<transport::Node> node;
  private: transport::Node::Publisher cmdVelPub;
};

GZ_ADD_PLUGIN(PatternMovementPlugin,
              System,
              PatternMovementPlugin::ISystemConfigure,
              PatternMovementPlugin::ISystemPreUpdate)

}  // namespace sim
}  // namespace gz