#pragma once

#include <deque>
#include <memory>

#ifndef Q_MOC_RUN

// TODO(Martin-Idel-SI): Reenable once available
// #include <tf/message_filter.h>
// #include "nav_msgs/msg/odometry.hpp"
#endif

#include "dynamic_slam_interfaces/msg/multi_object_odometry_path.hpp"
#include "dynamic_slam_interfaces/msg/object_odometry_path.hpp"
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

class ObjectOdometryPathDisplay : public
  rviz_common::MessageFilterDisplay<MultiObjectOdometryPath>
{
  Q_OBJECT
  public:
  // TODO(Martin-Idel-SI): Constructor for testing, remove once ros_nodes can be mocked and call
  // initialize() instead
  explicit ObjectOdometryPathDisplay(rviz_common::DisplayContext * context);
  ObjectOdometryPathDisplay();
  ~ObjectOdometryPathDisplay() override;

  /** @brief Overridden from Display. */
  void reset() override;

  /** @brief Overridden from MessageFilterDisplay. */
  void processMessage(MultiObjectOdometryPath::ConstSharedPtr msg) override;

protected:
  /** @brief Overridden from Display. */
  void onInitialize() override;

private Q_SLOTS:
  void updateBufferLength();
  void updateLineWidth();
  void updatePoseStyle();
  void updatePoseAxisGeometry();
  void updatePoseArrowGeometry();

private:
  void destroyDisplays();

  struct PropertyStruct {
      // pose marker property
    rviz_common::properties::EnumProperty * pose_style_property_;
    rviz_common::properties::FloatProperty * pose_axes_length_property_;
    rviz_common::properties::FloatProperty * pose_axes_radius_property_;
    rviz_common::properties::FloatProperty * pose_arrow_shaft_length_property_;
    rviz_common::properties::FloatProperty * pose_arrow_head_length_property_;
    rviz_common::properties::FloatProperty * pose_arrow_shaft_diameter_property_;
    rviz_common::properties::FloatProperty * pose_arrow_head_diameter_property_;
    rviz_common::properties::FloatProperty * line_width_property_;

  };


  struct SingleDisplay : public PropertyStruct  {
    std::vector<rviz_rendering::BillboardLine *> billboard_lines_;
    std::vector<std::vector<rviz_rendering::Axes *>> axes_chain_;
    std::vector<std::vector<rviz_rendering::Arrow *>> arrow_chain_;
    Ogre::SceneNode* scene_node_;
    Ogre::SceneManager* scene_manager_;

    void destroyObjects();
    void destroyPoseAxesChain();
    void destroyPoseArrowChain();

    void updateBufferLength(size_t buffer_length);

    void updateBillBoardLine(
      rviz_rendering::BillboardLine * billboard_line, 
      const ObjectOdometryPath& msg,
      const Ogre::Matrix4 & transform);

      void updateAxesMarkers(
        std::vector<rviz_rendering::Axes *> & axes_vect, 
        const ObjectOdometryPath& msg,
        const Ogre::Matrix4 & transform);
      void updateArrowMarkers(
        std::vector<rviz_rendering::Arrow *> & arrow_vect, 
        const ObjectOdometryPath& msg,
        const Ogre::Matrix4 & transform);

    void updatePoseMarkers(size_t buffer_index, 
      const ObjectOdometryPath& msg,
      const Ogre::Matrix4 & transform);

    void allocateArrowVector(std::vector<rviz_rendering::Arrow *> & arrow_vect, size_t num);
    void allocateAxesVector(std::vector<rviz_rendering::Axes *> & axes_vect, size_t num);
  };

  SingleDisplay* getSingleDisplay(const ObjectOdometryPath& msg);

  //map of objects id/path segments to displays
  std::map<std::pair<int, int>, SingleDisplay> object_data_;

  rviz_common::properties::IntProperty * buffer_length_property_;
  PropertyStruct properties;

  enum PoseStyle
  {
    NONE,
    AXES,
    ARROWS,
  };
};

}  // namespace rviz_dynamic_slam_plugins

