// Copyright 2026 Muhammad Ali Sadiq (github.com/turtleshelp)
// SPDX-License-Identifier: Apache-2.0

// Grasp assist for Gazebo Classic.
//
// Why: the OpenMANIPULATOR-X finger links weigh 1 g. When they squeeze a
// cube while the arm is moved by position commands, ODE becomes unstable and
// the cube is thrown away (seen in tests: a finger reported 0.077 m on a
// joint limited to 0.019 m). This plugin is the usual Gazebo Classic
// workaround (same idea as the GraspFix plugin): once the gripper has closed
// on the object, the object is attached to the gripper link with a fixed
// joint, and released again on request.
//
// It never attaches an object that is not actually between the jaws: the
// attach service first checks the object's position in the gripper frame and
// refuses if it is outside a small box around the grasp point.
//
// Services (std_srvs/Trigger), in the plugin's ROS namespace:
//   attach  - attach <object_model> to <gripper_link> if it is in the jaws
//   detach  - remove the fixed joint

#include <memory>
#include <boost/thread/recursive_mutex.hpp>
#include <sstream>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace mobile_robot_description
{

class GraspAssistPlugin : public gazebo::WorldPlugin
{
public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    world_ = world;
    node_ = gazebo_ros::Node::Get(sdf);

    robot_model_ = Get<std::string>(sdf, "robot_model", "mobile_robot");
    gripper_link_ = Get<std::string>(sdf, "gripper_link", "link5");
    object_model_ = Get<std::string>(sdf, "object_model", "pickup_cube");
    grasp_point_ = Get<ignition::math::Vector3d>(
      sdf, "grasp_point", ignition::math::Vector3d(0.126, 0.0, 0.0));
    tolerance_ = Get<ignition::math::Vector3d>(
      sdf, "tolerance", ignition::math::Vector3d(0.03, 0.015, 0.02));

    attach_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "attach",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {Attach(res);});

    detach_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "detach",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {Detach(res);});

    RCLCPP_INFO(
      node_->get_logger(), "Grasp assist ready: %s/%s <-> %s",
      robot_model_.c_str(), gripper_link_.c_str(), object_model_.c_str());
  }

private:
  template<typename T>
  static T Get(sdf::ElementPtr sdf, const std::string & key, const T & def)
  {
    return sdf->HasElement(key) ? sdf->Get<T>(key) : def;
  }

  void Attach(std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    boost::recursive_mutex::scoped_lock lock(
      *world_->Physics()->GetPhysicsUpdateMutex());

    if (joint_) {
      res->success = true;
      res->message = "already attached";
      return;
    }

    auto robot = world_->ModelByName(robot_model_);
    auto object = world_->ModelByName(object_model_);
    if (!robot || !object) {
      res->success = false;
      res->message = "robot or object model not found";
      return;
    }

    auto gripper = robot->GetLink(gripper_link_);
    auto object_link = object->GetLinks().empty() ? nullptr : object->GetLinks()[0];
    if (!gripper || !object_link) {
      res->success = false;
      res->message = "gripper link or object link not found";
      return;
    }

    // Object position in the gripper frame, relative to the grasp point
    const auto rel = gripper->WorldPose().Inverse() * object_link->WorldPose();
    const auto off = rel.Pos() - grasp_point_;

    std::ostringstream info;
    info.precision(3);
    info << std::fixed << "object offset from grasp point (gripper frame): ["
         << off.X() << ", " << off.Y() << ", " << off.Z() << "] m";

    if (std::abs(off.X()) > tolerance_.X() ||
      std::abs(off.Y()) > tolerance_.Y() ||
      std::abs(off.Z()) > tolerance_.Z())
    {
      res->success = false;
      res->message = "object is not between the jaws; " + info.str();
      return;
    }

    joint_ = world_->Physics()->CreateJoint("fixed", robot);
    joint_->Attach(gripper, object_link);
    joint_->Load(gripper, object_link, ignition::math::Pose3d());
    joint_->SetModel(robot);
    joint_->Init();

    res->success = true;
    res->message = "attached; " + info.str();
    RCLCPP_INFO(node_->get_logger(), "%s", res->message.c_str());
  }

  void Detach(std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    boost::recursive_mutex::scoped_lock lock(
      *world_->Physics()->GetPhysicsUpdateMutex());

    if (!joint_) {
      res->success = true;
      res->message = "nothing attached";
      return;
    }

    joint_->Detach();
    joint_.reset();
    res->success = true;
    res->message = "detached";
    RCLCPP_INFO(node_->get_logger(), "Object detached");
  }

  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::physics::JointPtr joint_;

  std::string robot_model_;
  std::string gripper_link_;
  std::string object_model_;
  ignition::math::Vector3d grasp_point_;
  ignition::math::Vector3d tolerance_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr attach_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr detach_srv_;
};

GZ_REGISTER_WORLD_PLUGIN(GraspAssistPlugin)

}  // namespace mobile_robot_description
