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
{
  setupProperties();
  context_ = display_context;
  scene_node_ = scene_node;
  scene_manager_ = context_->getSceneManager();
  updateTrajectoryBufferLength();
}

ObjectOdometryDisplay::ObjectOdometryDisplay()
{
  setupProperties();
}

ObjectOdometryDisplay::~ObjectOdometryDisplay() {
  destroyTrajectoryObjects();
}

void ObjectOdometryDisplay::setupProperties() {
  show_trajectory_property_ = new rviz_common::properties::BoolProperty(
    "Trajectory", true,
    "Whether or not to show the trajectory of all objects.",
    this, SLOT(updateShowTrajectory()), this);

  show_covariance_property_ = new rviz_common::properties::CovarianceProperty(
    "Covariance", true,
    "Whether or not the covariances of the messages should be shown.",
    this, SLOT(updateCovariances()));

  show_velocity_property_ = new rviz_common::properties::BoolProperty(
    "Show Velocity", true,
    "Whether or not to show the current velocity of all objects.",
    this, SLOT(updateShowVelocity()), this);

  show_only_last_pose_property_ = new rviz_common::properties::BoolProperty(
    "Show Only Final Pose", true,
    "Whether or not to only the final pose of each object. Indepenant of showing the trajectory.",
    this, SLOT(updateShowPoses()), this);

  style_property_ = new rviz_common::properties::EnumProperty(
    "Line Style", "Billboards",
    "The rendering operation to use to draw the grid lines.",
    show_trajectory_property_, SLOT(updateTrajectoryStyle()), this);
  style_property_->addOption("Lines", LINES);
  style_property_->addOption("Billboards", BILLBOARDS);

  buffer_length_property_ = new rviz_common::properties::IntProperty(
    "Buffer Length", 1,
    "Number of paths to display.",
    show_trajectory_property_, SLOT(updateTrajectoryBufferLength()), this);
  buffer_length_property_->setMin(1);


  show_object_label_property_ = new rviz_common::properties::BoolProperty(
    "Show Object Label", true,
    "Whether or not to show the object label of all objects.",
    this, SLOT(updateShowObjectLabel()), this);

  keep_property_ = new rviz_common::properties::IntProperty(
    "Keep", 100,
    "Number of arrows to keep before removing the oldest.  0 means keep all of them.",
    this);
  keep_property_->setMin(0);

  path_diameter_property_ = new rviz_common::properties::FloatProperty(
    "Diameter", 0.2, "Diamter (m) of the object paths.",
    show_trajectory_property_, SLOT(updatePathDiameter()), this);
  path_diameter_property_->setMin(0.001f);
  // path_diameter_property_->hide();


  axes_length_property_ = new rviz_common::properties::FloatProperty(
    "Axes Length", 1, "Length of each axis, in meters.",
    this, SLOT(updateAxisGeometry()), this);

  axes_radius_property_ = new rviz_common::properties::FloatProperty(
    "Axes Radius", 0.1f, "Radius of each axis, in meters.",
    this, SLOT(updateAxisGeometry()), this);
}


void ObjectOdometryDisplay::onInitialize() {
  MFDClass::onInitialize();
  updateTrajectoryBufferLength();
  // updateShapeChoice();
}


void ObjectOdometryDisplay::reset() {
  MFDClass::reset();
  updateTrajectoryBufferLength();
  clear();
}

void ObjectOdometryDisplay::update(float wall_dt, float ros_dt) {
  (void) wall_dt;
  (void) ros_dt;

  size_t keep = keep_property_->getInt();


  auto keep_per_object = [keep](ObjectMetaData& object_meta_data) -> void {
    if (keep > 0) {
      while (object_meta_data.axes_.size() > keep) {
        object_meta_data.axes_.pop_front();
        object_meta_data.covariances_.pop_front();
      }
    }
  };

  for(auto&[object_label, meta_data] : object_data_) {
     keep_per_object(meta_data);
  }

}

void ObjectOdometryDisplay::processMessage(dynamic_slam_interfaces::msg::ObjectOdometry::ConstSharedPtr msg) {
  // if (!messageIsValid(msg) || messageIsSimilarToPrevious(msg)) {
  //   return;
  // }
  // std::cout << "Here " <<std::endl;
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->transform(msg->odom.header, msg->odom.pose.pose, position, orientation)) {
    setMissingTransformToFixedFrame(msg->odom.header.frame_id);
    return;
  }

  // std::cout << "Made it " <<std::endl;
  setTransformOk();
  addObjectData(position, orientation, msg);
  queueRender();


}

void ObjectOdometryDisplay::onEnable() {
  MFDClass::onEnable();
  updateShapeVisibility();
}


void ObjectOdometryDisplay::clear() {
  object_data_.clear();

}

void ObjectOdometryDisplay::updateCovariances() {
  auto update_per_object = [&](ObjectMetaData& object_meta_data) {
    for (const auto & covariance : object_meta_data.covariances_) {
      covariance->updateUserData(show_covariance_property_->getUserData());
    }
  };
  for(auto&[_, object_meta_data] : object_data_) { update_per_object(object_meta_data); }

  context_->queueRender();

}

void ObjectOdometryDisplay::updateShowTrajectory() {
  bool show_traj = show_trajectory_property_->getBool();
  auto style = static_cast<LineStyle>(style_property_->getOptionInt());

  auto update_per_object = [&, show_traj, style](ObjectMetaData& object_meta_data) {
    switch (style) {
      case LINES:
        // for(auto manual_object : object_meta_data.manual_objects_) {
        //   if(manual_object) manual_object->
        // }
        // break;
        std::cout << "LINES not implemented" <<std::endl;
        break;

      case BILLBOARDS:  // billboards with configurable width
         for(auto billboard : object_meta_data.billboard_lines_) {
          if(billboard) billboard->getSceneNode()->setVisible(show_traj);
        }
        break;
    }
  };

  for(auto&[_, object_meta_data] : object_data_) { update_per_object(object_meta_data); }
}

void ObjectOdometryDisplay::updateShowVelocity() {
  // bool selectable = show_velocity_property_->getBool();
}

void ObjectOdometryDisplay::updateShowPoses() {
  bool show_only_last_pose = show_only_last_pose_property_->getBool();
  if(show_only_last_pose) {
    for(auto&[object_label, meta_data] : object_data_) {
      meta_data.showOnlyFinalPose();
    }
  }
}

void ObjectOdometryDisplay::updateShowObjectLabel() {
  bool show_label = show_object_label_property_->getBool();
  for(auto&[_, meta_data] : object_data_) {
      if(meta_data.object_label_) meta_data.object_label_->setVisible(show_label);
    }
}

void ObjectOdometryDisplay::updatePathDiameter() {
  auto style = static_cast<LineStyle>(style_property_->getOptionInt());
  float line_width = path_diameter_property_->getFloat();

  if (style == BILLBOARDS) {
    for(auto& [object_id, object_meta_data] : object_data_) {
      for (auto billboard_line : object_meta_data.billboard_lines_) {
        if (billboard_line) {
          billboard_line->setLineWidth(line_width);
        }
      }
    }
  }
  context_->queueRender();
}

void ObjectOdometryDisplay::updateTrajectoryStyle()
{
  auto style = static_cast<LineStyle>(style_property_->getOptionInt());

  if (style == BILLBOARDS) {
    path_diameter_property_->show();
  } else {
    path_diameter_property_->hide();
  }

  updateTrajectoryBufferLength();
}


// void ObjectOdometryDisplay::updateShapeChoice() {}

void ObjectOdometryDisplay::updateShapeVisibility() {}

// void ObjectOdometryDisplay::updateColorAndAlpha() {}

// void ObjectOdometryDisplay::updateArrowsGeometry() {}

void ObjectOdometryDisplay::updateAxisGeometry() {
  // for (const auto & axes : axes_) {
  //   updateAxes(axes);
  // }

}

void ObjectOdometryDisplay::updateTrajectoryBufferLength()
{
  // Destroy all path objects
  destroyTrajectoryObjects();

  // // Destroy all axes and arrows
  // destroyPoseAxesChain();
  // destroyPoseArrowChain();

  // Read options
  auto buffer_length = static_cast<size_t>(buffer_length_property_->getInt());
  auto style = static_cast<LineStyle>(style_property_->getOptionInt());

  //allocate new data for all existing objects, sincee we have destroyed all the data in destroyTrajectoryObjects
  for(auto&[_, object_meta_data] : object_data_) { allocateTrajectoryBuffer(object_meta_data,buffer_length,style); }

}

void ObjectOdometryDisplay::destroyTrajectoryObjects() {
  // Destroy all simple lines, if any
  auto destroy_per_object = [&](ObjectMetaData& object_meta_data) {
    for (auto manual_object : object_meta_data.manual_objects_) {
      manual_object->clear();
      scene_manager_->destroyManualObject(manual_object);
    }
    object_meta_data.manual_objects_.clear();

    // Destroy all billboards, if any
    for (auto billboard_line : object_meta_data.billboard_lines_) {
      delete billboard_line;  // also destroys the corresponding scene node
    }
    object_meta_data.billboard_lines_.clear();
  };

  for(auto&[_, object_meta_data] : object_data_) { destroy_per_object(object_meta_data); }
}

void ObjectOdometryDisplay::addObjectData(
  const Ogre::Vector3 & position, 
  const Ogre::Quaternion & orientation,
  dynamic_slam_interfaces::msg::ObjectOdometry::ConstSharedPtr msg)
{
  int object_label = msg->object_id;

  auto msg_colour = msg->colour;
  Ogre::ColourValue colour(msg_colour.r, msg_colour.g, msg_colour.b, 1);

  if (object_data_.find(object_label) == object_data_.end()) {
    auto buffer_length = static_cast<size_t>(buffer_length_property_->getInt());
    auto style = static_cast<LineStyle>(style_property_->getOptionInt());
    //new object
    ObjectMetaData new_object_data;
    //must allocate memory for new object when it is created!!
    allocateTrajectoryBuffer(new_object_data, buffer_length, style);

    const std::string label = "Object " + std::to_string(msg->object_id);
    new_object_data.object_label_ = new rviz_rendering::MovableText(label);
    scene_node_->attachObject(new_object_data.object_label_);
    new_object_data.object_label_->setVisible(show_object_label_property_->getBool());
    new_object_data.object_label_->setTextAlignment(
      rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_CENTER);
    new_object_data.object_label_->setColor(colour);
    object_data_.emplace(object_label, std::move(new_object_data));
  }


  ObjectMetaData& object_meta_data = object_data_.at(object_label);
  size_t buffer_index = object_meta_data.messages_received_ % buffer_length_property_->getInt();

  auto style = static_cast<LineStyle>(style_property_->getOptionInt());
  Ogre::ManualObject * manual_object = nullptr;
  rviz_rendering::BillboardLine * billboard_line = nullptr;

  // Ogre::Matrix4 transform(orientation);
  // transform.setTrans(position);

  // std::cout << buffer_index << " " << object_meta_data.billboard_lines_.size() << std::endl;

  object_meta_data.axes_.push_back(createAndSetAxes(position, orientation));
  object_meta_data.covariances_.push_back(createAndSetCovarianceVisual(position, orientation, msg));

  // Delete oldest element
  switch (style) {
    case LINES:
      manual_object = object_meta_data.manual_objects_[buffer_index];
      manual_object->clear();
      break;

    case BILLBOARDS:
      billboard_line = object_meta_data.billboard_lines_[buffer_index];
      billboard_line->clear();
      billboard_line->setNumLines(1);
      billboard_line->setMaxPointsPerLine(static_cast<uint32_t>(object_meta_data.axes_.size()));
      billboard_line->setLineWidth(path_diameter_property_->getFloat());

      // std::cout << colour.r << " " <<  colour.g << " " << colour.b << " " << colour.a << std::endl;

      for (const auto& axes : object_meta_data.axes_) {
        // Ogre::Vector3 xpos = transform * rviz_common::pointMsgToOgre(pose_stamped.pose.position);
        auto xpos = axes->getPosition();
        billboard_line->addPoint(xpos, colour);
      }
      break;
  }

  //update text location
  auto xpos = object_meta_data.axes_.back()->getPosition();
  object_meta_data.object_label_->setGlobalTranslation(xpos);

  updateShowPoses();
  updateShowTrajectory();
  // updateShowVelocity()
  updateShowObjectLabel();

  object_meta_data.messages_received_++;

}

void ObjectOdometryDisplay::allocateTrajectoryBuffer(ObjectMetaData& object_meta_data, size_t buffer_length, LineStyle style) {
  switch (style) {
      case LINES:  // simple lines with fixed width of 1px
        object_meta_data.manual_objects_.reserve(buffer_length);
        for (size_t i = 0; i < buffer_length; i++) {
          auto manual_object = scene_manager_->createManualObject();
          manual_object->setDynamic(true);
          scene_node_->attachObject(manual_object);

          object_meta_data.manual_objects_.push_back(manual_object);
        }
        break;

      case BILLBOARDS:  // billboards with configurable width
        object_meta_data.billboard_lines_.reserve(buffer_length);
        for (size_t i = 0; i < buffer_length; i++) {
          auto billboard_line = new rviz_rendering::BillboardLine(scene_manager_, scene_node_);
          object_meta_data.billboard_lines_.push_back(billboard_line);
        }
        break;
    }
}


std::unique_ptr<rviz_rendering::Axes> ObjectOdometryDisplay::createAndSetAxes(
    const Ogre::Vector3 & position, const Ogre::Quaternion & orientation) {

  auto axes = std::make_unique<rviz_rendering::Axes>(
    scene_manager_, scene_node_->createChildSceneNode(),
    axes_length_property_->getFloat(),
    axes_radius_property_->getFloat());
  axes->setPosition(position);
  axes->setOrientation(orientation);
  axes->getSceneNode()->setVisible(true);
  return axes;

}

 std::unique_ptr<rviz_rendering::CovarianceVisual> ObjectOdometryDisplay::createAndSetCovarianceVisual(
    const Ogre::Vector3 & position,
    const Ogre::Quaternion & orientation,
    dynamic_slam_interfaces::msg::ObjectOdometry::ConstSharedPtr message) 
{
  auto covariance_visual = std::make_unique<rviz_rendering::CovarianceVisual>(
    scene_manager_, scene_node_->createChildSceneNode());
  covariance_visual->setPosition(position);
  covariance_visual->setOrientation(orientation);
  auto quaternion = message->odom.pose.pose.orientation;
  covariance_visual->setCovariance(
    rviz_common::quaternionMsgToOgre(quaternion), message->odom.pose.covariance);
  covariance_visual->updateUserData(show_covariance_property_->getUserData());

  return covariance_visual;

}


void ObjectOdometryDisplay::ObjectMetaData::showOnlyFinalPose() {
   for(auto& axes : axes_) {
        axes->getSceneNode()->setVisible(false);
    }
    auto& last_pose = axes_.back();
    last_pose->getSceneNode()->setVisible(true);
}


} //rviz_dynamic_slam_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_dynamic_slam_plugins::ObjectOdometryDisplay, rviz_common::Display)
