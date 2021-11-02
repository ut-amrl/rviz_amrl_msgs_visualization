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

#include <memory>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "amrl_msgs/VisualizationMsg.h"
#include "rviz/message_filter_display.h"
#include "rviz/ogre_helpers/line.h"
#include "rviz/ogre_helpers/point_cloud.h"


namespace Ogre {
class SceneNode;
class Vector3;
class Quaternion;
}  // namespace Ogre

namespace rviz {
class ColorProperty;
class FloatProperty;
class IntProperty;
}  // namespace rviz

namespace amrl_visualization {

class Visual;
class AmrlVisualization: public 
    rviz::MessageFilterDisplay<amrl_msgs::VisualizationMsg> {
 Q_OBJECT
 public:
  AmrlVisualization();
  virtual ~AmrlVisualization();

 protected:
  virtual void onInitialize();

  virtual void reset();


 private Q_SLOTS:
  void updateColorAndAlpha();
  // void updateHistoryLength();

 private:
  void processMessage(const amrl_msgs::VisualizationMsg::ConstPtr& msg);
  void DrawLine(const uint32_t color, 
                const Eigen::Vector2f& p0,
                const Eigen::Vector2f& p1);

  rviz::FloatProperty* alpha_property_;

  // Lines from the AMRL Visualization message.
  std::vector<std::unique_ptr<rviz::Line>> lines_;

  // Points from the AMRL Visualization message.
  std::unique_ptr<rviz::PointCloud> points_;
};

}  // namespace amrl_visualization