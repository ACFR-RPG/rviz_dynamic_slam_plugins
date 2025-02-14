#include "rviz_dynamic_slam_plugins/object_odometry_display.hpp"

#include <memory>
#include <string>

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/axes.hpp"

#include "rviz_common/logging.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/covariance_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/validate_floats.hpp"

// #include "./quaternion_helper.hpp"

namespace rviz_dynamic_slam_plugins
{

ObjectOdometryDisplay::ObjectOdometryDisplay(
  rviz_common::DisplayContext * display_context, Ogre::SceneNode * scene_node)
: ObjectOdometryDisplay() 
{
  context_ = display_context;
  scene_node_ = scene_node;
  scene_manager_ = context_->getSceneManager();
}

ObjectOdometryDisplay::ObjectOdometryDisplay()
{
  show_covariance_property_ = new rviz_common::properties::CovarianceProperty(
    "Covariance", true,
    "Whether or not the covariances of the messages should be shown.",
    this, SLOT(updateCovariances()));

  show_velocity_property_ = new rviz_common::properties::BoolProperty(
    "Velocity", true,
    "Whether or not to show the current velocity of all objects.",
    this, SLOT(updateShowVelocity()), this);

  show_object_label_property_ = new rviz_common::properties::BoolProperty(
    "Object Label", true,
    "Whether or not to show the object label of all objects.",
    this, SLOT(updateShowObjectLabel()), this);
  
  axes_length_property_ = new rviz_common::properties::FloatProperty(
    "Axes Length", 0.5, "Length of each axis, in meters.",
    this, SLOT(updateAxisGeometry()), this);

  axes_radius_property_ = new rviz_common::properties::FloatProperty(
    "Axes Radius", 0.03f, "Radius of each axis, in meters.",
    this, SLOT(updateAxisGeometry()), this);

  keep_property_ = new rviz_common::properties::IntProperty(
    "Keep", 1,
    "Number of arrows to keep before removing the oldest.  0 means keep all of them.",
    this);
  keep_property_->setMin(0);
}

ObjectOdometryDisplay::~ObjectOdometryDisplay() {
  // reset();
  // destroyTrajectoryObjects();
}


void ObjectOdometryDisplay::onInitialize() {
  MFDClass::onInitialize();
  // updateTrajectoryBufferLength();
  // updateShapeChoice();
}


void ObjectOdometryDisplay::reset() {
  MFDClass::reset();
  clear();
}

void ObjectOdometryDisplay::onEnable() {
  MFDClass::onEnable();
  // updateShapeVisibility();
}


void ObjectOdometryDisplay::update(float wall_dt, float ros_dt) {
  (void) wall_dt;
  (void) ros_dt;

  size_t keep = keep_property_->getInt();


  auto keep_per_object = [keep](SingleDisplay& disp) -> void {
    if (keep > 0) {
      while (disp.axes_.size() > keep) {
        disp.axes_.pop_front();
        disp.covariances_.pop_front();
      }
    }
  };

  for(auto&[_, meta_data] : object_data_) {
     keep_per_object(meta_data);
  }

}

void ObjectOdometryDisplay::processMessage(dynamic_slam_interfaces::msg::ObjectOdometry::ConstSharedPtr msg) {
  // if (!messageIsValid(msg) || messageIsSimilarToPrevious(msg)) {
  //   return;
  // }
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->transform(msg->odom.header, msg->odom.pose.pose, position, orientation)) {
    setMissingTransformToFixedFrame(msg->odom.header.frame_id);
    return;
  }
  setTransformOk();
  SingleDisplay* display = getSingleDisplay(msg);

  display->createAndAddAxes(position, orientation, axes_length_property_->getFloat(), axes_radius_property_->getFloat());
  display->createAndAddCovarianceVisual(position, orientation, msg);
  display->object_label_scene_node_->setPosition(position);


  queueRender();
}



ObjectOdometryDisplay::SingleDisplay* ObjectOdometryDisplay::getSingleDisplay(ObjectOdometry::ConstSharedPtr msg) {
  int object_label = msg->object_id;

  SingleDisplay* display = nullptr;

  if (object_data_.find(object_label) == object_data_.end()) {
    SingleDisplay object_display;
    object_display.scene_node_ = scene_node_->createChildSceneNode();
    object_display.object_label_scene_node_ = scene_node_->createChildSceneNode();
    object_display.scene_manager_ = scene_manager_;
    object_display.show_covariance_property_ = show_covariance_property_;

    const std::string label = "object " + std::to_string(msg->object_id);
    object_display.object_label_ = new rviz_rendering::MovableText(label, "Liberation Sans", 0.1f);

    //attach object to object scene scene node
    object_display.object_label_scene_node_->attachObject(object_display.object_label_);
    object_display.object_label_->setTextAlignment(
      rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_CENTER);

    Ogre::ColourValue colour(0.23,0.159,0, 1);
    object_display.object_label_->setColor(colour);
    object_data_.emplace(object_label, std::move(object_display));
  }

  display = &object_data_.at(object_label);
  assert (display != nullptr);
  return display;
}


void ObjectOdometryDisplay::updateCovariances() {
  auto update_per_object = [&](SingleDisplay& disp) {
    for (const auto & covariance : disp.covariances_) {
      covariance->updateUserData(show_covariance_property_->getUserData());
    }
  };
  for(auto&[_, disp] : object_data_) { update_per_object(disp); }

  context_->queueRender();

}

void ObjectOdometryDisplay::updateShowVelocity() {
  // bool selectable = show_velocity_property_->getBool();
}

void ObjectOdometryDisplay::updateShowObjectLabel() {
  bool show_label = show_object_label_property_->getBool();
  for(auto&[_, disp] : object_data_) {
      if(disp.object_label_) disp.object_label_->setVisible(show_label);
    }
  context_->queueRender();

}


void ObjectOdometryDisplay::updateAxisGeometry() {
  auto update_per_object = [&](SingleDisplay& disp) {
    for (const auto & axes : disp.axes_) {
      axes->set(axes_length_property_->getFloat(), axes_radius_property_->getFloat());
    }
  };
  for(auto&[_, disp] : object_data_) { update_per_object(disp); }

  context_->queueRender();


}

void ObjectOdometryDisplay::SingleDisplay::createAndAddAxes(
  const Ogre::Vector3 & position, const Ogre::Quaternion & orientation,
  float length, float radius)
{
  auto axes = std::make_unique<rviz_rendering::Axes>(
    scene_manager_, scene_node_,
    length,
    radius);
  axes->setPosition(position);
  axes->setOrientation(orientation);
  axes->getSceneNode()->setVisible(true);

  axes_.push_back(std::move(axes));
}

void ObjectOdometryDisplay::SingleDisplay::createAndAddCovarianceVisual(
  const Ogre::Vector3 & position,
  const Ogre::Quaternion & orientation,
  ObjectOdometry::ConstSharedPtr message)
{
  auto covariance_visual = std::make_unique<rviz_rendering::CovarianceVisual>(
    scene_manager_, scene_node_);
  covariance_visual->setPosition(position);
  covariance_visual->setOrientation(orientation);
  auto quaternion = message->odom.pose.pose.orientation;
  covariance_visual->setCovariance(
    rviz_common::quaternionMsgToOgre(quaternion), message->odom.pose.covariance);
  covariance_visual->updateUserData(show_covariance_property_->getUserData());

  covariances_.push_back(std::move(covariance_visual));
}

void ObjectOdometryDisplay::clear() {
  auto update_per_object = [&](SingleDisplay& disp) {
    disp.axes_.clear();
    disp.covariances_.clear();

    if(disp.object_label_) {
      disp.object_label_scene_node_->detachObject(disp.object_label_);
      delete disp.object_label_;
    }
  };
  for(auto&[_, disp] : object_data_) { update_per_object(disp); }

  object_data_.clear();

}


} //rviz_dynamic_slam_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_dynamic_slam_plugins::ObjectOdometryDisplay, rviz_common::Display)
