#include "rviz_dynamic_slam_plugins/object_odometry_path_display.hpp"

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

ObjectOdometryPathDisplay::ObjectOdometryPathDisplay(rviz_common::DisplayContext * context)
: ObjectOdometryPathDisplay()
{
  context_ = context;
  scene_manager_ = context->getSceneManager();
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  updateBufferLength();
}

ObjectOdometryPathDisplay::ObjectOdometryPathDisplay()
{
  buffer_length_property_ = new rviz_common::properties::IntProperty(
    "Buffer Length", 1,
    "Number of paths to display.",
    this, SLOT(updateBufferLength()));
  buffer_length_property_->setMin(1);

  properties.line_width_property_ = new rviz_common::properties::FloatProperty(
    "Line Width", 0.03f,
    "The width, in meters, of each path line."
    "Only works with the 'Billboards' style.",
    this, SLOT(updateLineWidth()), this);
  properties.line_width_property_->setMin(0.001f);
  properties.line_width_property_->hide();

  properties.pose_style_property_ = new rviz_common::properties::EnumProperty(
    "Pose Style", "None",
    "Shape to display the pose as.",
    this, SLOT(updatePoseStyle()));
  properties.pose_style_property_->addOption("None", NONE);
  properties.pose_style_property_->addOption("Axes", AXES);
  properties.pose_style_property_->addOption("Arrows", ARROWS);

  properties.pose_axes_length_property_ = new rviz_common::properties::FloatProperty(
    "Length", 0.3f,
    "Length of the axes.",
    this, SLOT(updatePoseAxisGeometry()) );
    properties.pose_axes_radius_property_ = new rviz_common::properties::FloatProperty(
    "Radius", 0.03f,
    "Radius of the axes.",
    this, SLOT(updatePoseAxisGeometry()) );

  properties.pose_arrow_shaft_length_property_ = new rviz_common::properties::FloatProperty(
    "Shaft Length",
    0.1f,
    "Length of the arrow shaft.",
    this,
    SLOT(updatePoseArrowGeometry()));
  properties.pose_arrow_head_length_property_ = new rviz_common::properties::FloatProperty(
    "Head Length", 0.2f,
    "Length of the arrow head.",
    this,
    SLOT(updatePoseArrowGeometry()));
  properties.pose_arrow_shaft_diameter_property_ = new rviz_common::properties::FloatProperty(
    "Shaft Diameter",
    0.1f,
    "Diameter of the arrow shaft.",
    this,
    SLOT(updatePoseArrowGeometry()));
  properties.pose_arrow_head_diameter_property_ = new rviz_common::properties::FloatProperty(
    "Head Diameter",
    0.3f,
    "Diameter of the arrow head.",
    this,
    SLOT(updatePoseArrowGeometry()));
  properties.pose_axes_length_property_->hide();
  properties.pose_axes_radius_property_->hide();
  properties.pose_arrow_shaft_length_property_->hide();
  properties.pose_arrow_head_length_property_->hide();
  properties.pose_arrow_shaft_diameter_property_->hide();
  properties.pose_arrow_head_diameter_property_->hide();
}

ObjectOdometryPathDisplay::~ObjectOdometryPathDisplay()
{
  destroyDisplays();
}

void ObjectOdometryPathDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateBufferLength();
}

void ObjectOdometryPathDisplay::reset()
{
  MFDClass::reset();
  updateBufferLength();
}



void ObjectOdometryPathDisplay::processMessage(dynamic_slam_interfaces::msg::MultiObjectOdometryPath::ConstSharedPtr msg)
{
  // // Calculate index of oldest element in cyclic buffer
  size_t bufferIndex = messages_received_ % buffer_length_property_->getInt();
  auto buffer_length = static_cast<size_t>(buffer_length_property_->getInt());
  std::cout << "Gotten msg " << std::endl;

  rviz_rendering::BillboardLine * billboard_line = nullptr;
  //create object per path
  for(const auto& path : msg->paths) {

    SingleDisplay* display = nullptr;
    const auto object_id = path.object_id;
    if (object_data_.find(object_id) == object_data_.end()) {
      std::cout << "making new object " << object_id << std::endl;

      SingleDisplay object_display;
      object_display.scene_node_ = scene_node_->createChildSceneNode();
      object_display.scene_manager_ = scene_manager_;
      object_display.pose_style_property_ = properties.pose_style_property_;
      object_display.pose_axes_length_property_ = properties.pose_axes_length_property_;
      object_display.pose_axes_radius_property_ = properties.pose_axes_radius_property_;
      object_display.pose_arrow_shaft_length_property_ = properties.pose_arrow_shaft_length_property_;
      object_display.pose_arrow_head_length_property_ = properties.pose_arrow_head_length_property_;
      object_display.pose_arrow_shaft_diameter_property_ = properties.pose_arrow_shaft_diameter_property_;
      object_display.pose_arrow_head_diameter_property_ = properties.pose_arrow_head_diameter_property_;
      object_display.line_width_property_ = properties.line_width_property_;

      object_display.updateBufferLength(buffer_length);


      object_data_.emplace(object_id, std::move(object_display));

    }

    std::cout << "gotten new object " << object_id << std::endl;


    display = &object_data_.at(object_id);
    assert (display != nullptr);

    std::cout << "gotten disp " << object_id << std::endl;

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!context_->getFrameManager()->getTransform(path.header, position, orientation)) {
      setMissingTransformToFixedFrame(path.header.frame_id);
      return;
    }

    std::cout << "gotten position and orientation " << object_id << std::endl;
    setTransformOk();

    Ogre::Matrix4 transform(orientation);
    transform.setTrans(position);

    billboard_line = display->billboard_lines_[bufferIndex];
    assert (billboard_line != nullptr);
    billboard_line->clear();

    std::cout << "gotten billboard_line " << object_id << std::endl;

    display->updateBillBoardLine(billboard_line, path, transform);
    display->updatePoseMarkers(bufferIndex, path, transform);

    std::cout << "dome updaet " << object_id << std::endl;

  }
  context_->queueRender();


}

void ObjectOdometryPathDisplay::updateBufferLength() {
  auto buffer_length = static_cast<size_t>(buffer_length_property_->getInt());
  for(auto& [_, obj] : object_data_) {
    obj.updateBufferLength(buffer_length);
  }
}

void ObjectOdometryPathDisplay::destroyDisplays() {
  for(auto& [_, obj] : object_data_) {
    obj.destroyObjects();
    obj.destroyPoseAxesChain();
    obj.destroyPoseArrowChain();
  }

  object_data_.clear();
}


void ObjectOdometryPathDisplay::updateLineWidth() {
  float line_width = properties.line_width_property_->getFloat();

  for(auto& [_, obj] : object_data_) {
    for (auto billboard_line : obj.billboard_lines_) {
      if (billboard_line) {
        billboard_line->setLineWidth(line_width);
      }
    }
  }


  context_->queueRender();
}

void ObjectOdometryPathDisplay::updatePoseStyle() {
  auto pose_style = static_cast<PoseStyle>(properties.pose_style_property_->getOptionInt());
  switch (pose_style) {
    case AXES:
      properties.pose_axes_length_property_->show();
      properties.pose_axes_radius_property_->show();
      properties.pose_arrow_shaft_length_property_->hide();
      properties.pose_arrow_head_length_property_->hide();
      properties.pose_arrow_shaft_diameter_property_->hide();
      properties.pose_arrow_head_diameter_property_->hide();
      break;
    case ARROWS:
      properties.pose_axes_length_property_->hide();
      properties.pose_axes_radius_property_->hide();
      properties.pose_arrow_shaft_length_property_->show();
      properties.pose_arrow_head_length_property_->show();
      properties.pose_arrow_shaft_diameter_property_->show();
      properties.pose_arrow_head_diameter_property_->show();
      break;
    default:
      properties.pose_axes_length_property_->hide();
      properties.pose_axes_radius_property_->hide();
      properties.pose_arrow_shaft_length_property_->hide();
      properties.pose_arrow_head_length_property_->hide();
      properties.pose_arrow_shaft_diameter_property_->hide();
      properties.pose_arrow_head_diameter_property_->hide();
  }
  updateBufferLength();
}

void ObjectOdometryPathDisplay::updatePoseAxisGeometry() {
  for(auto& [_, obj] : object_data_) {
    for (auto & axes_vect : obj.axes_chain_) {
      for (auto & axes : axes_vect) {
        axes->set(
          properties.pose_axes_length_property_->getFloat(),
          properties.pose_axes_radius_property_->getFloat() );
      }
    }
  }
  context_->queueRender();
}

void ObjectOdometryPathDisplay::updatePoseArrowGeometry() {
  for(auto& [_, obj] : object_data_) {
    for (auto & arrow_vect : obj.arrow_chain_) {
      for (auto & arrow : arrow_vect) {
        arrow->set(
          properties.pose_arrow_shaft_length_property_->getFloat(),
          properties.pose_arrow_shaft_diameter_property_->getFloat(),
          properties.pose_arrow_head_length_property_->getFloat(),
          properties.pose_arrow_head_diameter_property_->getFloat());
      }
    }
  }
  context_->queueRender();
}

void ObjectOdometryPathDisplay::SingleDisplay::updateBufferLength(size_t buffer_length) {
  // Destroy all path objects
  destroyObjects();

  // Destroy all axes and arrows
  destroyPoseAxesChain();
  destroyPoseArrowChain();

  billboard_lines_.reserve(buffer_length);
  for (size_t i = 0; i < buffer_length; i++) {
    auto billboard_line = new rviz_rendering::BillboardLine(scene_manager_, scene_node_);
    billboard_lines_.push_back(billboard_line);
  }

  axes_chain_.resize(buffer_length);
  arrow_chain_.resize(buffer_length);

}

void ObjectOdometryPathDisplay::SingleDisplay::allocateAxesVector(std::vector<rviz_rendering::Axes *> & axes_vect, size_t num)
{
  auto vector_size = axes_vect.size();
  if (num > vector_size) {
    axes_vect.reserve(num);
    for (auto i = vector_size; i < num; ++i) {
      axes_vect.push_back(
        new rviz_rendering::Axes(
          scene_manager_, scene_node_,
          pose_axes_length_property_->getFloat(),
          pose_axes_radius_property_->getFloat()));
    }
  } else if (num < vector_size) {
    for (auto i = num; i < vector_size; ++i) {
      delete axes_vect[i];
    }
    axes_vect.resize(num);
  }
}

void ObjectOdometryPathDisplay::SingleDisplay::allocateArrowVector(std::vector<rviz_rendering::Arrow *> & arrow_vect, size_t num)
{
  auto vector_size = arrow_vect.size();
  if (num > vector_size) {
    arrow_vect.reserve(num);
    for (auto i = vector_size; i < num; ++i) {
      arrow_vect.push_back(new rviz_rendering::Arrow(scene_manager_, scene_node_));
    }
  } else if (num < vector_size) {
    for (auto i = num; i < vector_size; ++i) {
      delete arrow_vect[i];
    }
    arrow_vect.resize(num);
  }
}

void ObjectOdometryPathDisplay::SingleDisplay::destroyPoseAxesChain()
{
  for (auto & axes_vect : axes_chain_) {
    allocateAxesVector(axes_vect, 0);
  }
  axes_chain_.clear();
}

void ObjectOdometryPathDisplay::SingleDisplay::destroyPoseArrowChain()
{
  for (auto & arrow_vect : arrow_chain_) {
    allocateArrowVector(arrow_vect, 0);
  }
  arrow_chain_.clear();
}

void ObjectOdometryPathDisplay::SingleDisplay::destroyObjects() {
  for (auto billboard_line : billboard_lines_) {
    delete billboard_line;  // also destroys the corresponding scene node
  }
  billboard_lines_.clear();
}


void ObjectOdometryPathDisplay::SingleDisplay::updateBillBoardLine(
  rviz_rendering::BillboardLine * billboard_line, const ObjectOdometryPath& msg,
  const Ogre::Matrix4 & transform)
{

  auto msg_colour = msg.colour;
  Ogre::ColourValue colour(msg_colour.r, msg_colour.g, msg_colour.b, 1);
  
  auto num_points = msg.object_odometries.size();
  billboard_line->setNumLines(1);
  billboard_line->setMaxPointsPerLine(static_cast<uint32_t>(num_points));
  billboard_line->setLineWidth(line_width_property_->getFloat());

  for (size_t i = 0; i < num_points; ++i) {
    const geometry_msgs::msg::Point & pos = msg.object_odometries[i].odom.pose.pose.position;
    Ogre::Vector3 xpos = transform * rviz_common::pointMsgToOgre(pos);
    billboard_line->addPoint(xpos, colour);
  }
}

void ObjectOdometryPathDisplay::SingleDisplay::updatePoseMarkers(
  size_t buffer_index, const ObjectOdometryPath& msg, const Ogre::Matrix4 & transform)
{
  auto pose_style = static_cast<PoseStyle>(pose_style_property_->getOptionInt());
  auto & arrow_vect = arrow_chain_[buffer_index];
  auto & axes_vect = axes_chain_[buffer_index];

  if (pose_style == AXES) {
    updateAxesMarkers(axes_vect, msg, transform);
  }
  if (pose_style == ARROWS) {
    updateArrowMarkers(arrow_vect, msg, transform);
  }
}

void ObjectOdometryPathDisplay::SingleDisplay::updateAxesMarkers(
  std::vector<rviz_rendering::Axes *> & axes_vect, const ObjectOdometryPath& msg,
  const Ogre::Matrix4 & transform)
{
  auto num_points = msg.object_odometries.size();
  allocateAxesVector(axes_vect, num_points);
  for (size_t i = 0; i < num_points; ++i) {
    const geometry_msgs::msg::Point & pos = msg.object_odometries[i].odom.pose.pose.position;
    axes_vect[i]->setPosition(transform * rviz_common::pointMsgToOgre(pos));
    Ogre::Quaternion orientation(rviz_common::quaternionMsgToOgre( msg.object_odometries[i].odom.pose.pose.orientation));

    // Extract the rotation part of the transformation matrix as a quaternion
    Ogre::Quaternion transform_orientation = transform.linear();

    axes_vect[i]->setOrientation(transform_orientation * orientation);
  }
}

void ObjectOdometryPathDisplay::SingleDisplay::updateArrowMarkers(
  std::vector<rviz_rendering::Arrow *> & arrow_vect, const ObjectOdometryPath& msg,
  const Ogre::Matrix4 & transform)
{
  auto num_points = msg.object_odometries.size();
  allocateArrowVector(arrow_vect, num_points);
  for (size_t i = 0; i < num_points; ++i) {
    // QColor color = pose_arrow_color_property_->getColor();
    // arrow_vect[i]->setColor(color.redF(), color.greenF(), color.blueF(), 1.0f);

    arrow_vect[i]->set(
      pose_arrow_shaft_length_property_->getFloat(),
      pose_arrow_shaft_diameter_property_->getFloat(),
      pose_arrow_head_length_property_->getFloat(),
      pose_arrow_head_diameter_property_->getFloat());
    const geometry_msgs::msg::Point & pos = msg.object_odometries[i].odom.pose.pose.position;
    arrow_vect[i]->setPosition(transform * rviz_common::pointMsgToOgre(pos));
    Ogre::Quaternion orientation(rviz_common::quaternionMsgToOgre(msg.object_odometries[i].odom.pose.pose.orientation));

    // Extract the rotation part of the transformation matrix as a quaternion
    Ogre::Quaternion transform_orientation = transform.linear();

    Ogre::Vector3 dir(1, 0, 0);
    dir = transform_orientation * orientation * dir;
    arrow_vect[i]->setDirection(dir);
  }
}


} //rviz_dynamic_slam_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_dynamic_slam_plugins::ObjectOdometryPathDisplay, rviz_common::Display)
