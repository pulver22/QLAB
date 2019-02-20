/*
  The MIT License (MIT)
  Copyright (c) 2017 Riccardo Polvara

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  #MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
  #CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  #SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

  Class offering various methods not related with other classes
*/

#include "../include/utilities.h"

Utilities::Utilities()
{
}

Utilities::~Utilities()
{
}

double Utilities::assignReward(geometry_msgs::Pose quadrotor_pose, BoundingBox bb_land, BoundingBox bb_flight, bool *done)
{
  double reward = 0;

  //std::cout << "QuadRotor:" << quadrotor_pose.position.x << " " << quadrotor_pose.position.y << " " << quadrotor_pose.position.z << std::endl;
  //std::cout << "Done: " << done << std::endl;
  *done = false;
  if (quadrotor_pose.position.x >= bb_flight.getMinX() && quadrotor_pose.position.x <= bb_flight.getMaxX() && quadrotor_pose.position.y >= bb_flight.getMinY() && quadrotor_pose.position.y <= bb_flight.getMaxY() && quadrotor_pose.position.z >= bb_flight.getMinZ() && quadrotor_pose.position.z <= bb_flight.getMaxZ())
  {
    if (quadrotor_pose.position.x >= bb_land.getMinX() && quadrotor_pose.position.x <= bb_land.getMaxX())
    {
      if (quadrotor_pose.position.y >= bb_land.getMinY() && quadrotor_pose.position.y <= bb_land.getMaxY())
      {
        if (quadrotor_pose.position.z >= bb_land.getMinZ() && quadrotor_pose.position.z <= bb_land.getMaxZ())
        {
          //TODO:Use this code only for landing
          //        // UAV is inside the bounding box
          //        if (done == true)
          //        {
          //          reward = 1;  // reward = 1 if UAV landed
          //        }
          //        else
          //        {
          //          reward = 0;  // reward = 0 if UAV still flying
          //        }
          *done = true;
          reward = 1.0; //UAV inside the 3D bb_land
        }
        else
        {
          reward = -0.01; // UAV centered on X and Y but not on Z
        }
      }
      else
      {
        reward = -0.01; // UAV centered on X but not on Y
      }
    }
    else
    {
      reward = -0.01; // UAV totally outside the bb_land
    }
  }
  else
  {
    reward = -1.0; // UAV outside the bb_flight
    *done = true;
  }
  //std::cout<< "Reward: "<< reward << std::endl;
  return reward;
}

double Utilities::assignRewardWithoutFlightBB(geometry_msgs::Pose quadrotor_pose, BoundingBox bb_land, BoundingBox bb_flight, bool *done)
{
  double reward = 0;

  //std::cout << "QuadRotor:" << quadrotor_pose.position.x << " " << quadrotor_pose.position.y << " " << quadrotor_pose.position.z << std::endl;
  //std::cout << "Done: " << done << std::endl;
  *done = false;
  if (quadrotor_pose.position.x >= bb_land.getMinX() && quadrotor_pose.position.x <= bb_land.getMaxX())
  {
    if (quadrotor_pose.position.y >= bb_land.getMinY() && quadrotor_pose.position.y <= bb_land.getMaxY())
    {
      if (quadrotor_pose.position.z >= bb_land.getMinZ() && quadrotor_pose.position.z <= bb_land.getMaxZ())
      {
        //TODO:Use this code only for landing
        //        // UAV is inside the bounding box
        //        if (done == true)
        //        {
        //          reward = 1;  // reward = 1 if UAV landed
        //        }
        //        else
        //        {
        //          reward = 0;  // reward = 0 if UAV still flying
        //        }
        *done = true;
        reward = 1.0; //UAV inside the 3D bb_land
      }
      else
      {
        reward = -0.01; // UAV centered on X and Y but not on Z
      }
    }
    else
    {
      reward = -0.01; // UAV centered on X but not on Y
    }
  }
  else
  {
    reward = -0.01; // UAV totally outside the bb_land
  }

  //std::cout<< "Reward: "<< reward << std::endl;
  return reward;
}

double Utilities::assignRewardWithoutFlightBB(geometry_msgs::Pose quadrotor_pose, BoundingBox bb_land, BoundingBox bb_flight, bool *done, std::string action, bool *wrong_altitude)
{
  double reward = 0;
  *done = false;
  reward = -0.01;
  *wrong_altitude = false;

  // NOTE: assign a positive or a negative reward only if the action taken is "descend""
  //if (action.compare("descend") == 0)
  //{

  if (quadrotor_pose.position.z  > 15.3)
  {
    //std::cout << "[ERROR] Wrong altitude: " << quadrotor_pose.position.z << std::endl;
    *wrong_altitude = true;
  }
  else if (quadrotor_pose.position.z < 14.8 && quadrotor_pose.position.z > 14.3)
  {
    //std::cout << "[ERROR] Wrong altitude: " << quadrotor_pose.position.z << std::endl;
    *wrong_altitude = true;
  }
  else if (quadrotor_pose.position.z < 13.8 && quadrotor_pose.position.z > 13.3)
  {
    //std::cout << "[ERROR] Wrong altitude: " << quadrotor_pose.position.z << std::endl;
    *wrong_altitude = true;
  }
  else if (quadrotor_pose.position.z < 12.8 && quadrotor_pose.position.z > 12.3)
  {
    //std::cout << "[ERROR] Wrong altitude: " << quadrotor_pose.position.z << std::endl;
    *wrong_altitude = true;
  }
  else if (quadrotor_pose.position.z < 11.8 && quadrotor_pose.position.z > 11.3)
  {
    //std::cout << "[ERROR] Wrong altitude: " << quadrotor_pose.position.z << std::endl;
    *wrong_altitude = true;
  }
  else if (quadrotor_pose.position.z < 10.8 && quadrotor_pose.position.z > 10.3)
  {
    //std::cout << "[ERROR] Wrong altitude: " << quadrotor_pose.position.z << std::endl;
    *wrong_altitude = true;
  }
  else if (quadrotor_pose.position.z < 9.8 && quadrotor_pose.position.z > 9.3)
  {
    //std::cout << "[ERROR] Wrong altitude: " << quadrotor_pose.position.z << std::endl;
    *wrong_altitude = true;
  }
  else if (quadrotor_pose.position.z < 8.8 && quadrotor_pose.position.z > 8.3)
  {
    //std::cout << "[ERROR] Wrong altitude: " << quadrotor_pose.position.z << std::endl;
    *wrong_altitude = true;
  }
  else if (quadrotor_pose.position.z < 7.8 && quadrotor_pose.position.z > 7.3)
  {
    //std::cout << "[ERROR] Wrong altitude: " << quadrotor_pose.position.z << std::endl;
    *wrong_altitude = true;
  }
  else if (quadrotor_pose.position.z < 6.8 && quadrotor_pose.position.z > 6.3)
  {
    //std::cout << "[ERROR] Wrong altitude: " << quadrotor_pose.position.z << std::endl;
    *wrong_altitude = true;
  }
  else if (quadrotor_pose.position.z < 5.8 && quadrotor_pose.position.z > 5.3)
  {
    //std::cout << "[ERROR] Wrong altitude: " << quadrotor_pose.position.z << std::endl;
    *wrong_altitude = true;
  }
  else if (quadrotor_pose.position.z < 4.8 && quadrotor_pose.position.z > 4.3)
  {
    //std::cout << "[ERROR] Wrong altitude: " << quadrotor_pose.position.z << std::endl;
    *wrong_altitude = true;
  }
  else if (quadrotor_pose.position.z < 3.8 && quadrotor_pose.position.z > 3.3)
  {
    //std::cout << "[ERROR] Wrong altitude: " << quadrotor_pose.position.z << std::endl;
    *wrong_altitude = true;
  }
  else if (quadrotor_pose.position.z < 2.8 && quadrotor_pose.position.z > 2.3)
  {
    //std::cout << "[ERROR] Wrong altitude: " << quadrotor_pose.position.z << std::endl;
    *wrong_altitude = true;
  }
  else
  {
    *wrong_altitude = false;
  }

  if (quadrotor_pose.position.z >= bb_land.getMinZ() && quadrotor_pose.position.z <= bb_land.getMaxZ())
  {
    if (quadrotor_pose.position.x >= bb_land.getMinX() && quadrotor_pose.position.x <= bb_land.getMaxX())
    {
      if (quadrotor_pose.position.y >= bb_land.getMinY() && quadrotor_pose.position.y <= bb_land.getMaxY())
      {
        *done = true;
          return 1.0; //UAV inside the 3D bb_land
      }
      else
      {
        *done = true;
        return -1.0; //UAV centered on Z and X but not on Y
      }
    }
    else
    {
      *done = true;
      return -1.0; //centered on Z but not on X (and Y
    }
  }
  //}

  return reward;
}

double Utilities::assignRewardWhenLanding(geometry_msgs::Pose quadrotor_pose, BoundingBox bb_land, BoundingBox bb_flight, bool *done, std::string action)
{
  // For every action different from landing, reward is -0.01 and done is set to false
  double reward = -0.01;
  *done = false;

  if (action.compare("land") == 0)
  {
    reward = -1.0; // general negative reward for landing outside the landing_BB
    *done = true;  // done is equal to true only when the UAV is landing
    if (quadrotor_pose.position.x >= bb_land.getMinX() && quadrotor_pose.position.x <= bb_land.getMaxX())
    {
      if (quadrotor_pose.position.y >= bb_land.getMinY() && quadrotor_pose.position.y <= bb_land.getMaxY())
      {
        if (quadrotor_pose.position.z >= bb_land.getMinZ() && quadrotor_pose.position.z <= bb_land.getMaxZ())
        {
          reward = 1.0; //UAV inside the landing_BB
        }
      }
    }
  }

  // If the UAV is too low we want to finish the episode
  if (quadrotor_pose.position.z <= 0.3)
  {
    *done = true;
    reward = -1.0;
  }
  return reward;
}
