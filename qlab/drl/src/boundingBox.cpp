/*The MIT License (MIT)
Copyright (c) 2017 Riccardo Polvara

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
#MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
#CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Implementation of a 3D bounding box, a cubic area in which the UAV should be spawn or land.
*/

#include "../include/boundingBox.h"
#include "ros/ros.h"

BoundingBox::BoundingBox()
{
}

BoundingBox::~BoundingBox()
{
}

BoundingBox::BoundingBox(geometry_msgs::Pose origin, double half_size, double height)
{
  this->min_x_ = origin.position.x - half_size;
  this->max_x_ = origin.position.x + half_size;
  this->min_y_ = origin.position.y - half_size;
  this->max_x_ = origin.position.y + half_size;
  this->min_z_ = origin.position.z;
  this->max_z_ = origin.position.z + height;
}

void BoundingBox::setDimension(geometry_msgs::Pose origin, double half_size, double height)
{
  this->min_x_ = origin.position.x - half_size;
  this->max_x_ = origin.position.x + half_size;
  this->min_y_ = origin.position.y - half_size;
  this->max_y_ = origin.position.y + half_size;
  this->min_z_ = origin.position.z;
  this->max_z_ = origin.position.z + height;
  //ROS_INFO("BoundingBox's edge:\n minX:%f maxX:%f minY:%f maxY:%f minZ:%f maxZ:%f", this->min_x_, this->max_x_, this->min_y_, this->max_y_, this->min_z_, this->max_z_);
}

double BoundingBox::getMinX()
{
  return this->min_x_;
}
double BoundingBox::getMaxX()
{
  return this->max_x_;
}
double BoundingBox::getMinY()
{
  return this->min_y_;
}
double BoundingBox::getMaxY()
{
  return this->max_y_;
}
double BoundingBox::getMinZ()
{
  return this->min_z_;
}
double BoundingBox::getMaxZ()
{
  return this->max_z_;
}
