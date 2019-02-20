/*The MIT License (MIT)
Copyright (c) 2017 Riccardo Polvara

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
#MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
#CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Class offering various methods not related with other classes
*/

#ifndef UTILITIES_H
#define UTILITIES_H

#include "./boundingBox.h"
#include "geometry_msgs/Pose.h"
#include <string>

// Declaration of a swiss army knive class offering multiple methods
class Utilities
{
public:
  Utilities();
  ~Utilities();

  /*

    Given the UAV and two BB's pose, generate a reward,

    @param quadrotor_pose is the UAV's pose (for the moment, only the position expressed as x,y,z)
    @param bb_land is the BB in which the UAV has to land
    @param bb_flight is the BB in which the UAV can fly and in which it must be respawn

    @return an integer reward = 1 in the UAV is located inside the bb_land, -1 otherwise

  */
  double assignReward(geometry_msgs::Pose quadrotor_pose, BoundingBox bb_land, BoundingBox bb_flight, bool *done);
  double assignRewardWithoutFlightBB(geometry_msgs::Pose quadrotor_pose, BoundingBox bb_land, BoundingBox bb_flight, bool *done);
  double assignRewardWithoutFlightBB(geometry_msgs::Pose quadrotor_pose, BoundingBox bb_land, BoundingBox bb_flight, bool *done, std::string action, bool *wrong_altitude);
  double assignRewardWhenLanding(geometry_msgs::Pose quadrotor_pose, BoundingBox bb_land, BoundingBox bb_flight, bool *done, std::string action);

protected:
};
#endif
