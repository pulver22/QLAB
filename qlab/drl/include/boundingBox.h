/*The MIT License (MIT)
Copyright (c) 2017 Riccardo Polvara

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
#MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
#CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include "geometry_msgs/Pose.h"

//Declaration of a 3D bounding box, a cubic area in which the UAV should be spawn or land.
class BoundingBox
{
public:
  BoundingBox();
  ~BoundingBox();

  /*
    Create a square 3D BB

    @param origin is the centre of the base of the BB
    @param half_size is half of the side of the BB
  */
  BoundingBox(geometry_msgs::Pose origin, double half_size, double height);

  /*
    Modify the size of the position of the BB

    @param origin is the centre of the base of the BB
    @param half_size is half of the side of the BB
  */
  void setDimension(geometry_msgs::Pose origin, double half_size, double height);
  double getMinX();
  double getMaxX();
  double getMinY();
  double getMaxY();
  double getMinZ();
  double getMaxZ();

protected:
  double min_x_;
  double min_y_;
  double max_x_;
  double max_y_;
  double min_z_;
  double max_z_;
};

#endif
