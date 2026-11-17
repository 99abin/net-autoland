// Simulation-only, one-shot finite catapult. No ROS, MAVLink, arming or modes.
#include <cmath>
#include <chrono>
#include <mutex>
#include <sstream>
#include <string>

#include <gz/common/Console.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace autoland
{
class LaunchAssist final : public gz::sim::System,
                           public gz::sim::ISystemConfigure,
                           public gz::sim::ISystemPreUpdate
{
 public: void Configure(const gz::sim::Entity &,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        gz::sim::EntityComponentManager &,
                        gz::sim::EventManager &) override
  {
    this->duration = _sdf->Get<double>("duration_s", 0.75).first;
    this->forward = _sdf->Get<double>("forward_force_n", 120.0).first;
    this->upward = _sdf->Get<double>("upward_force_n", 80.0).first;
    this->maxSpeed = _sdf->Get<double>("maximum_initial_speed_mps", 2.0).first;
    this->maxHeight = _sdf->Get<double>("maximum_initial_height_m", 0.35).first;
    this->modelName = _sdf->Get<std::string>("model_name", "merlinv2").first;
    this->linkName = _sdf->Get<std::string>("link_name", "body").first;
    this->prefix = _sdf->Get<std::string>(
      "service_prefix", "/world/merlin_landing/launch_assist").first;
    if (!(std::isfinite(this->duration) && this->duration > 0 && this->duration <= 1.0 &&
          std::isfinite(this->forward) && this->forward > 0 && this->forward <= 200 &&
          std::isfinite(this->upward) && this->upward >= 0 && this->upward <= 100 &&
          std::isfinite(this->maxSpeed) && this->maxSpeed > 0 && this->maxSpeed <= 2 &&
          std::isfinite(this->maxHeight) && this->maxHeight > 0 && this->maxHeight <= 0.5))
    {
      gzerr << "LaunchAssist: invalid force/time/ground limits; disabled\n";
      return;
    }
    this->valid = this->transport.Advertise(this->prefix + "/trigger",
                                          &LaunchAssist::Trigger, this) &&
                  this->transport.Advertise(this->prefix + "/status",
                                          &LaunchAssist::Status, this);
    gzmsg << "LaunchAssist: one-shot simulator catapult available at " << this->prefix << '\n';
  }

 public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                        gz::sim::EntityComponentManager &_ecm) override
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->valid)
      return;
    if (this->body == gz::sim::kNullEntity)
    {
      auto model = _ecm.EntityByComponents(gz::sim::components::Model(),
                                         gz::sim::components::Name(this->modelName));
      if (model == gz::sim::kNullEntity)
        return;
      this->body = gz::sim::Model(model).LinkByName(_ecm, this->linkName);
      if (this->body == gz::sim::kNullEntity)
        return;
      gz::sim::Link(this->body).EnableVelocityChecks(_ecm);
    }
    auto link = gz::sim::Link(this->body);
    const auto velocity = link.WorldLinearVelocity(_ecm);
    const auto pose = gz::sim::worldPose(this->body, _ecm);
    this->speed = velocity ? velocity->Length() : 1e9;
    this->height = pose.Pos().Z();
    const bool groundReady = velocity && this->speed <= this->maxSpeed &&
      this->height >= 0 && this->height <= this->maxHeight &&
      pose.Pos().X()*pose.Pos().X() + pose.Pos().Y()*pose.Pos().Y() <= 4 &&
      std::abs(pose.Rot().Roll()) <= 0.262 && std::abs(pose.Rot().Pitch()) <= 0.350;
    this->ready = groundReady && !_info.paused && !this->used;
    if (!this->used)
      this->state = _info.paused ? "PAUSED" : (this->ready ? "READY" : "WAITING_FOR_GROUND");
    if (_info.paused)
    {
      if (this->pending)
      {
        this->pending = false;
        this->state = "CANCELLED_PAUSED";
      }
      return;
    }
    if (this->pending)
    {
      this->pending = false;
      if (!groundReady)
      {
        this->state = "CANCELLED_GROUND_GUARD";
        return;
      }
      const double yaw = pose.Rot().Yaw();
      this->force.Set(this->forward * std::cos(yaw),
                      this->forward * std::sin(yaw), this->upward);
      this->startTime = _info.simTime;
      this->endTime = this->startTime + std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(std::chrono::duration<double>(this->duration));
      this->active = true;
      this->state = "APPLYING";
    }
    if (this->active)
    {
      if (_info.simTime < this->startTime || _info.simTime >= this->endTime)
      {
        this->active = false;
        this->state = "COMPLETE";
      }
      else
      {
        // Model origin is all-up CG. Force is reapplied for this step only;
        // no persistent wrench remains after the finite simulation-time interval.
        link.AddWorldForce(_ecm, this->force, gz::math::Vector3d::Zero);
        ++this->appliedSteps;
      }
    }
  }

 private: bool Trigger(const gz::msgs::Boolean &_request, gz::msgs::Boolean &_response)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    const bool accept = _request.data() && this->ready && !this->used;
    if (accept)
    {
      this->used = true;
      this->ready = false;
      this->pending = true;
      this->state = "QUEUED";
    }
    _response.set_data(accept);
    return true;
  }

 private: bool Status(const gz::msgs::Empty &, gz::msgs::StringMsg &_response)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    std::ostringstream json;
    json << "{\"ready\":" << (this->ready ? "true" : "false")
         << ",\"used\":" << (this->used ? "true" : "false")
         << ",\"active\":" << (this->active ? "true" : "false")
         << ",\"state\":\"" << this->state << "\",\"duration_s\":" << this->duration
         << ",\"height_m\":" << this->height << ",\"speed_mps\":" << this->speed
         << ",\"applied_steps\":" << this->appliedSteps << "}";
    _response.set_data(json.str());
    return true;
  }

 private: gz::transport::Node transport;
 private: std::mutex mutex;
 private: gz::sim::Entity body{gz::sim::kNullEntity};
 private: std::string modelName, linkName, prefix, state{"WAITING_FOR_GROUND"};
 private: double duration{0.75}, forward{120}, upward{80}, maxSpeed{2}, maxHeight{0.35};
 private: double height{0}, speed{0};
 private: std::chrono::steady_clock::duration startTime{}, endTime{};
 private: bool valid{false}, ready{false}, used{false}, pending{false}, active{false};
 private: unsigned int appliedSteps{0};
 private: gz::math::Vector3d force;
};
}

GZ_ADD_PLUGIN(autoland::LaunchAssist, gz::sim::System,
              autoland::LaunchAssist::ISystemConfigure,
              autoland::LaunchAssist::ISystemPreUpdate)
