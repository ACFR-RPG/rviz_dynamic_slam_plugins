#pragma once

#include <deque>
#include <memory>

#ifndef Q_MOC_RUN

// TODO(Martin-Idel-SI): Reenable once available
// #include <tf/message_filter.h>
// #include "nav_msgs/msg/odometry.hpp"
#endif

#include "dynamic_slam_interfaces/msg/object_odometry.hpp"
#include "dynamic_slam_interfaces/message_traits.hpp"

#include "rviz_rendering/objects/covariance_visual.hpp"
#include "rviz_common/message_filter_display.hpp"
#include "rviz_rendering/objects/billboard_line.hpp"
#include "rviz_rendering/objects/movable_text.hpp"


namespace rviz_rendering
{
class Arrow;
class Axes;
}  // namespace rviz_rendering

namespace rviz_common
{
namespace properties
{
class BoolProperty;
class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
class CovarianceProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_dynamic_slam_plugins
{

class ObjectOdometryDisplay : public
  rviz_common::MessageFilterDisplay<dynamic_slam_interfaces::msg::ObjectOdometry>
{
  Q_OBJECT

public:
  enum Shape
  {
    ArrowShape,
    AxesShape,
  };

//   // TODO(Martin-Idel-SI): Constructor for testing, remove once ros_nodes can be mocked and call
//   // initialize instead
  ObjectOdometryDisplay(rviz_common::DisplayContext * display_context, Ogre::SceneNode * scene_node);

  ObjectOdometryDisplay();

  ~ObjectOdometryDisplay() override;

  void onInitialize() override;

  void reset() override;

  // Overides of Display
  void update(float wall_dt, float ros_dt) override;

  void processMessage(dynamic_slam_interfaces::msg::ObjectOdometry::ConstSharedPtr msg) override;

protected:
  /** @brief Overridden from MessageFilterDisplay to get Arrow/Axes visibility correct. */
  void onEnable() override;

public Q_SLOTS:
  void updateCovariances();

private Q_SLOTS:
  void updateShowTrajectory();
  void updateShowVelocity();
  void updateShowPoses();
  void updateShowObjectLabel();
  void updatePathDiameter();
  void updateTrajectoryStyle();
  void updateTrajectoryBufferLength();
  void updateShapeVisibility();
  void updateAxisGeometry();

private:
  void setupProperties();

  // void updateArrow(const std::unique_ptr<rviz_rendering::Arrow> & arrow);

  // void updateAxes(const std::unique_ptr<rviz_rendering::Axes> & axes);
  void destroyTrajectoryObjects();
  

  struct ObjectMetaData {
    //TODO: what if non adjacent temporal data?


    //make temporal data struct internal

    //currentrly scene node not being descroyed!! (should be detroyed from the scene manager!!)
    Ogre::SceneNode* scene_node;
    std::deque<std::unique_ptr<rviz_rendering::Axes>> axes_;
    std::deque<std::unique_ptr<rviz_rendering::CovarianceVisual>> covariances_;

    std::vector<Ogre::ManualObject *> manual_objects_;
    std::vector<rviz_rendering::BillboardLine *> billboard_lines_;
    rviz_rendering::MovableText* object_label_;

    size_t messages_received_ = 0u;

    void showOnlyFinalPose();
  }; 

  //! Contains all renderable rviz-objects for each dynamic object
  std::map<int, ObjectMetaData> object_data_;

  void addObjectData(
    const Ogre::Vector3 & position, 
    const Ogre::Quaternion & orientation,
    dynamic_slam_interfaces::msg::ObjectOdometry::ConstSharedPtr msg);

  // bool messageIsValid(nav_msgs::msg::Odometry::ConstSharedPtr message);

  // bool messageIsSimilarToPrevious(nav_msgs::msg::Odometry::ConstSharedPtr message);


  std::unique_ptr<rviz_rendering::Axes> createAndSetAxes(
    const Ogre::Vector3 & position, const Ogre::Quaternion & orientation);

  std::unique_ptr<rviz_rendering::CovarianceVisual> createAndSetCovarianceVisual(
    const Ogre::Vector3 & position,
    const Ogre::Quaternion & orientation,
     dynamic_slam_interfaces::msg::ObjectOdometry::ConstSharedPtr message);

  void clear();

  enum LineStyle
  {
    LINES,
    BILLBOARDS
  };

  //allocates without destroying (used for new objects and impl updateTrajectoryBufferLength)
  void allocateTrajectoryBuffer(ObjectMetaData& object_meta_data, size_t buffer_length, LineStyle style);



  // rviz_common::properties::EnumProperty * shape_property_;
  rviz_common::properties::BoolProperty* show_trajectory_property_;
  rviz_common::properties::CovarianceProperty * show_covariance_property_;
  rviz_common::properties::BoolProperty* show_velocity_property_;
  rviz_common::properties::BoolProperty* show_only_last_pose_property_;

  // object label propertires
  rviz_common::properties::BoolProperty* show_object_label_property_;
  // rviz_common::properties::FloatProperty* object_label_z_offset_;

   
  // trajectoy (if shown) properties
  rviz_common::properties::IntProperty * keep_property_;
  rviz_common::properties::FloatProperty* path_diameter_property_;
  rviz_common::properties::IntProperty * buffer_length_property_;
  rviz_common::properties::EnumProperty * style_property_;

  //size of the pose axis
  rviz_common::properties::FloatProperty * axes_length_property_;
  rviz_common::properties::FloatProperty * axes_radius_property_;
  rviz_common::properties::FloatProperty * position_tolerance_property_;
  rviz_common::properties::FloatProperty * angle_tolerance_property_;

};

}  // namespace rviz_dynamic_slam_plugins

