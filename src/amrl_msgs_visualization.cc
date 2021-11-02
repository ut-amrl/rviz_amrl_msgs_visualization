// Copyright (c) 2021 Joydeep Biswas joydeepb@cs.utexas.edu

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in 
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#include "eigen3/Eigen/Dense"
#include "OGRE/OgreSceneNode.h"
#include "OGRE/OgreSceneManager.h"
#include "tf/transform_listener.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/frame_manager.h"

#include "std_msgs/Bool.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "rviz/message_filter_display.h"

#include "amrl_msgs_visualization.h"

using Ogre::Vector3;
using Eigen::Vector2f;

namespace amrl_visualization {


AmrlVisualization::AmrlVisualization() {
  alpha_property_ = new rviz::FloatProperty( 
      "Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.",
      this, SLOT(updateColorAndAlpha()));
}

AmrlVisualization::~AmrlVisualization() {}

void AmrlVisualization::onInitialize() {
  MFDClass::onInitialize();
}

void AmrlVisualization::reset() {
  MFDClass::reset();
  lines_.clear();
  points_.release();
}

void AmrlVisualization::updateColorAndAlpha() {}

void AmrlVisualization::DrawLine(const uint32_t color, const Vector2f& p0, const Vector2f& p1) {
  lines_.push_back(std::make_unique<rviz::Line>(
        context_->getSceneManager(), scene_node_));
    const float r = (color & 0xFF0000) / 255.0;
    const float g = (color & 0xFF00) / 255.0;
    const float b = (color & 0xFF) / 255.0;
    const float a = (color & 0xFF000000) / 255.0;
    lines_.back()->setColor(r, g, b, a);
    lines_.back()->setPoints(
        Vector3(p0.x(), p0.y(), 0),
        Vector3(p1.x(), p1.y(), 0));
}

void AmrlVisualization::processMessage(
    const amrl_msgs::VisualizationMsg::ConstPtr& msg) {
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;

  if (msg->header.frame_id != "map") {
    ROS_INFO("Non-map frame messages ignored");
    return;
  }
  if (!context_->getFrameManager()->getTransform(
      msg->header.frame_id, msg->header.stamp, position, orientation )) {
    ROS_INFO("Error transforming from frame '%s' to frame '%s'",
               msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }
  ROS_INFO("Received message: %s, %f", msg->header.frame_id.c_str(), msg->header.stamp.toSec());
  lines_.clear();
  DrawLine(0xFFFF0000, {0, 0}, {10, 10});

  for (const amrl_msgs::ColoredLine2D& l : msg->lines) {
    DrawLine(l.color, {l.p0.x, l.p0.y} , {l.p1.x, l.p1.y});
    printf("Line: %.3f,%.3f : %.3f,%.3f c:0x%08X\n",
        l.p0.x, l.p0.y, l.p1.x, l.p1.y, l.color);
  }
}

}  // namespace amrl_visualization



// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(amrl_visualization::AmrlVisualization, rviz::Display)