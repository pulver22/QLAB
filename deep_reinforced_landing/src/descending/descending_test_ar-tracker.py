#!/usr/bin/env python

# The MIT License (MIT)
# Copyright (c) 2017 Riccardo Polvara
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
# FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
# OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE # SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#
# DQN tensorflow implementation for achieving autonomous landing.
import random as STDrandom
import numpy as np
import sys
import datetime
import time
import os.path
# Adding these two lines solved the crash of Tesla K40
import os
os.environ["CUDA_VISIBLE_DEVICES"] = "0"
from deep_reinforced_landing.srv import NewCameraService, GetDoneAndReward, SendCommand, ResetPosition, GetRelativePose
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import GetModelState
import rospy
# Rename to avoid confusion with Image lib

GAZEBO_MODEL_PATH = "~/.gazebo/models/"


def get_relative_pose(ground_index, episode, step, reward, ground_list):
    """
    Get the relative pose of the quadrotor to the marker.

    @return a log containing information such as the ground used, episode, step, reward and the relative pose of the quadrotor
    """
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_relative_pose_proxy = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)
        resp = get_relative_pose_proxy('quadrotor', 'world')

        with open("./relative_distances_" + str(datetime.date.today().strftime("%m%d%Y")) + ".csv", "a") as myfile:
            x = resp.pose.position.x
            y = resp.pose.position.y
            z = resp.pose.position.z
            if abs(x) < 1.5 and abs(y) < 1.5 and abs(z) < 3.0:
                reward = 1
            else:
                reward = 0
            string_to_add = ground_list[ground_index] + "," + str(episode) + "," + str(
                step) + "," + str(reward) + "," + str(x) + "," + str(y) + "," + str(z) + '\n'
            myfile.write(string_to_add)
    except rospy.ServiceException, ex:
        print "Service call get_relative_pose failed: %e" % ex


def choose_random_ground(ground_list):
    """
    Return the name of the new ground to select
    """
    ground_index = STDrandom.randint(0, len(ground_list) - 1)
    ground = ground_list[ground_index]
    return ground


def remove_model(model):
    """
    Remove the model from the world. 

    @param model is the name of the model to remove
    """
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        remove_model_proxy = rospy.ServiceProxy(
            '/gazebo/delete_model', DeleteModel)
        remove_model_proxy(model)
    except rospy.ServiceException, ex:
        print "Service call delete_model failed: %e" % ex


def clean_world(ground_list):
    for ground in ground_list:
        try:
            ground = ground + "_plane"
            remove_model(ground)
        except:
            print "Error: Impossible to remove the " + str(ground)


def generate_new_world(model_to_add, ground_list):
    """
    Remove the old model on the floor and add new ones.

    @param model_to_remove is the name of the model to remove (all its istances)
    @param model_to_add is the name of the model to add
    """
    if type(model_to_add) is int:
        # print "Is an int!"
        model_to_add = ground_list[model_to_add]
        print "Ground choosen is " + str(model_to_add)

    # Spawn new istances for the new model
    os.system("rosrun gazebo_ros spawn_model -file ~/.gazebo/models/" +
              model_to_add + "/model.sdf -sdf -model " + model_to_add + "_plane -x 0 -y 0")


def main():
    """
    Main function for training the DQN network in learning how to accomplish autonomous landing.
    """
    ground_list = ["asphalt11", "asphalt12", "asphalt13",
                   "brick11", "brick12", "brick13",
                   "grass11", "grass12", "grass13",
                   "pavement11", "pavement12", "pavement13",
                   "sand11", "sand12", "sand13",
                   "snow11", "snow12", "snow13",
                   "soil11", "soil12", "soil13"]

    start_episode = 0  # change to last episode number

    tot_episodes = 210
    steps_per_episodes = 30  # expressed in number step
    noop_time = 2.0  # pause in seconds between actions

    num_ground_plane = 21
    episodes_per_ground = tot_episodes / num_ground_plane
    actual_ground_index = 0
    rospy.init_node("DeepReinforcedLanding")
    rospy.loginfo("----- Deep Reinforced Landing Node -----")

    # start here
    # 2. Call the artracker
    print "Calling the artracker"
    # try:
    #     #thread.start()
    #     subprocess.Popen(['terminator','-e', "roslaunch ardrone_tf_controller bottomcamera_autopilot.launch"])
    # except:
    #     print "Error: unable to start thread"

    for episode in range(start_episode, tot_episodes):
        # Reset the ground in the environment
        ground_index = episode / episodes_per_ground
        if ground_index != actual_ground_index or (episode == start_episode):
            clean_world(ground_list)
            generate_new_world(ground_index, ground_list)
        actual_ground_index = ground_index
        print("")
        print("Episode: " + str(episode))
        # 1-Accumulate the first state
        # reset_pose()
        # command = "rostopic pub /gazebo/set_model_state gazebo_msgs/ModelState \"model_name: 'quadrotor'\
        # pose:\
        #   position:\
        #     x: 0.0\
        #     y: 0.0\
        #     z: 20.0\
        #   orientation:\
        #     x: 0.0\
        #     y: 0.0\
        #     z: 0.0\
        #     w: 0.0\
        # twist:\
        #   linear:\
        #     x: 0.0\
        #     y: 0.0\
        #     z: 0.0\
        #   angular:\
        #     x: 0.0\
        #     y: 0.0\
        #     z: 0.0\
        # reference_frame: 'world'\" "

        # Clear every existing commands and send hovering
        os.system('rosservice call /drone_autopilot/clearCommands "{}"')
        os.system('rosservice call /drone_autopilot/hover "{}"')

        xy = np.random.normal(0.0, 0.75, 2)
        x = xy[0]
        y = xy[1]
        z = np.random.uniform(2.5, 20, 1)
        z = z[0]
        # x = STDrandom.uniform(0, 1.5)
        # sign = STDrandom.random()
        # if sign < 0.5:
        #     x = -x
        # y = STDrandom.uniform(0, 1.5)
        # sign = STDrandom.random()
        # if sign < 0.5:
        #     y = -y
        command = "rostopic pub -1 /takeoff std_msgs/Empty"
        os.system(command)
        command = "rostopic pub -1 /gazebo/set_model_state gazebo_msgs/ModelState '{model_name: 'quadrotor',\
        pose: {position: {x: " + str(x) + ", y: " + str(y) + ", z: " + str(z) + "}, orientation:{ x: 0.0, y: 0.0, z: 0.0, w: 0.0}},\
        twist: {linear: { x: 0.0, y: 0.0, z: 0.0}, angular:{ x: 0.0, y: 0.0, z: 0.0}},\
        reference_frame: world}'"
        os.system(command)
        print "Position resetted!"
        step = 0
        for step in range(steps_per_episodes):
            # To make the test fair, each step lasts 2 seconds like when using
            # the DRL algorithm
            print "Step: " + str(step)
            rospy.sleep(noop_time)
        # 4. Get relative pose of the marker
        get_relative_pose(ground_index, episode, step, "AR", ground_list)


if __name__ == "__main__":
    main()
