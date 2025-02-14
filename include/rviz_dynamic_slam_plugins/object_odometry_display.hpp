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

using namespace dynamic_slam_interfaces::msg;

class ObjectOdometryDisplay : public
  rviz_common::MessageFilterDisplay<dynamic_slam_interfaces::msg::ObjectOdometry>
{
  Q_OBJECT

public:

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
  void updateShowVelocity();
  void updateShowObjectLabel();
  void updateAxisGeometry();

private:
  rviz_common::properties::CovarianceProperty * show_covariance_property_;
  rviz_common::properties::BoolProperty* show_velocity_property_;
  rviz_common::properties::IntProperty * keep_property_;


  // object label propertires
  rviz_common::properties::BoolProperty* show_object_label_property_;

  //size of the pose axis
  rviz_common::properties::FloatProperty * axes_length_property_;
  rviz_common::properties::FloatProperty * axes_radius_property_;

  struct SingleDisplay {
    Ogre::SceneNode* scene_node_;
    Ogre::SceneManager* scene_manager_;
    Ogre::SceneNode* object_label_scene_node_;

    rviz_rendering::MovableText* object_label_;
    std::deque<std::unique_ptr<rviz_rendering::Axes>> axes_;
    std::deque<std::unique_ptr<rviz_rendering::CovarianceVisual>> covariances_;
    rviz_common::properties::CovarianceProperty * show_covariance_property_;

    void createAndAddAxes(
      const Ogre::Vector3 & position, const Ogre::Quaternion & orientation,
    float length, float radius);
  
    void createAndAddCovarianceVisual(
      const Ogre::Vector3 & position,
      const Ogre::Quaternion & orientation,
      ObjectOdometry::ConstSharedPtr message);
  

  };

  void clear();

  SingleDisplay* getSingleDisplay(ObjectOdometry::ConstSharedPtr msg);

  //object id to display
  std::map<int, SingleDisplay> object_data_;


};

}  // namespace rviz_dynamic_slam_plugins

