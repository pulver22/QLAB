#!/usr/bin/env python
#
#  The MIT License (MIT)
#  Copyright (c) 2017 Riccardo Polvara
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  Class for testing human performances while landing an UAV from a fixed
#  height of 20 meters.
import random as STDrandom
import numpy as np
import sys
import select
import tty
import termios
import datetime
import time
import os.path
import cv2
# Adding these two lines solved the crash of Tesla K40
from deep_reinforced_landing.srv import (NewCameraService, GetDoneAndReward,
                                         SendCommand, ResetPosition,
                                         GetRelativePose)  # DRL services
from gazebo_msgs.srv import DeleteModel  # Gazebo service for removing a model
import rospy
import subprocess  # needed for using bash command


def clean_world(ground_list):
    for ground in ground_list:
        try:
            ground = ground + "_plane"
            remove_model(ground)
        except:
            pass


def generate_new_world(model_to_add, ground_list):
    """
    Remove the old model on the floor and add new ones.

    @param model_to_remove is the name of the model to remove (all its istances)
    @param model_to_add is the name of the model to add
    """

    if type(model_to_add) is int:
        # print "Is an int!"
        model_to_add = ground_list[model_to_add]

    print "\n"
    rospy.logwarn("Ground choosen is " + str(model_to_add))
    # Spawn new istances for the new model
    os.system("rosrun gazebo_ros spawn_model -file ~/.gazebo/models/" +
              model_to_add + "/model.sdf -sdf -model " + model_to_add + "_plane -x 0 -y 0")


def generate_new_world_small(ground_list):
    """
    Remove the old model on the floor and add new ones.

    @param model_to_remove is the name of the model to remove (all its istances)
    @param model_to_add is the name of the model to add
    """

    model_to_add_array = np.random.randint(0, len(ground_list), 25)
    print "\n"
    # rospy.logwarn("Ground choosen is " + str(model_to_add_array))
    # Spawn new istances for the new model
    #os.system("rosrun gazebo_ros spawn_model -file ~/.gazebo/models/" + model_to_add +"/model.sdf -sdf -model " + model_to_add + "_plane -x 0 -y 0")
    coord_xy_list = [(a, b) for a in range(-20, 30, 10)
                     for b in range(-20, 30, 10)]

    for i, coord_xy in enumerate(coord_xy_list):
        os.system("rosrun gazebo_ros spawn_model -file ~/.gazebo/models/" +
                  ground_list[model_to_add_array[i]] + "/model.sdf -sdf -model " + str(i) + "_plane -x " + str(coord_xy[0]) + " -y " + str(coord_xy[1]))

    print "Waiting 5 sec"
    time.sleep(10)
    print "Waiting done!"


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


def get_done_reward():
    """
    Get the done status and the reward after completing an action.

    @return resp contains the reward and the done status
    """
    rospy.wait_for_service('/drl/get_done_reward')

    try:
        get_done_reward_proxy = rospy.ServiceProxy(
            '/drl/get_done_reward', GetDoneAndReward)
        resp = get_done_reward_proxy()
        return resp
    except rospy.ServiceException, ex:
        print "Service call get_reward_done failed: %e" % ex


def reset_pose():
    """
    Reset the UAV's randomly inside the flight Bounding Box.
    """
    rospy.wait_for_service('/drl/set_model_state')
    try:
        reset_pose_proxy = rospy.ServiceProxy(
            '/drl/set_model_state', ResetPosition)
        reset_pose_proxy(True)
    except rospy.ServiceException, ex:
        print "Service call reset_pose failed: %s" % ex


def send_action(action):
    """
    Send an action to the UAV.
    """
    rospy.wait_for_service('/drl/send_command')
    try:
        get_random_action_proxy = rospy.ServiceProxy(
            '/drl/send_command', SendCommand)
        # Call the service
        get_random_action_proxy(action)
    except rospy.ServiceException, ex:
        print "Service call get_random_action failed: %s" % ex


def get_relative_pose(ground_index, episode, step, reward):
    """
    Get the relative pose of the quadrotor to the marker.

    @return a log containing information such as the ground used, episode, step, reward and the relative pose of the quadrotor
    """
    rospy.wait_for_service('/drl/get_relative_pose')
    try:
        get_relative_pose_proxy = rospy.ServiceProxy(
            '/drl/get_relative_pose', GetRelativePose)
        resp = get_relative_pose_proxy()

        with open("./relative_distances_" + str(datetime.date.today().strftime("%m%d%Y")) + ".csv", "a") as myfile:
            ground_list = ["asphalt11", "asphalt12", "asphalt13",
                           "brick11", "brick12", "brick13",
                           "grass11", "grass12", "grass13",
                           "pavement11", "pavement12", "pavement13",
                           "sand11", "sand12", "sand13",
                           "snow11", "snow12", "snow13",
                           "soil11", "soil12", "soil13"]

            string_to_add = ground_list[ground_index] + "," + str(episode) + "," + str(step) + "," + str(reward) + "," + str(
                resp.pose.position.x) + "," + str(resp.pose.position.y) + "," + str(resp.pose.position.z) + '\n'
            myfile.write(string_to_add)
    except rospy.ServiceException, ex:
        print "Service call get_relative_pose failed: %e" % ex


def main():
    """
    Main function for the human pilot experiments.
    """
    start_episode_training = 0
    tot_episode_training = 5
    # Every episodes last maximum 40 seconds
    time_episode = 40.0
    tot_steps = 20
    rospy.init_node("DRLHumanTest")
    rospy.loginfo("----- DRL Human Test Node -----")
    start_time = time.time()
    # To change when resuming-----
    do_training = True
    start_episode_testing = 0
    tot_episode_testing = 48
    actual_ground_index = 0
    episode_per_ground = 2
    total_reward = 0
    already_generated = False
    #-----------------------------
    ground_list_testing = ["asphalt11", "asphalt12", "asphalt13",
                           "brick11", "brick12", "brick13",
                           "grass11", "grass12", "grass13",
                           "pavement11", "pavement12", "pavement13",
                           "sand11", "sand12", "sand13",
                           "snow11", "snow12", "snow13",
                           "soil11", "soil12", "soil13"]
    ground_list_testing_small = ["asphalt11_small", "asphalt12_small", "asphalt13_small",
                                 "brick11_small", "brick12_small", "brick13_small",
                                 "grass11_small", "grass12_small", "grass13_small",
                                 "pavement11_small", "pavement12_small", "pavement13_small",
                                 "sand11_small", "sand12_small", "sand13_small",
                                 "snow11_small", "snow12_small", "snow13_small",
                                 "soil11_small", "soil12_small", "soil13_small"]
    ground_list_traing = ["asphalt1", "asphalt2", "asphalt3", "asphalt4", "asphalt5", "asphalt6", "asphalt7", "asphalt8", "asphalt9", "asphalt10",
                          "brick1", "brick2", "brick3", "brick4", "brick5", "brick6", "brick7", "brick8", "brick9", "brick10",
                          "grass1", "grass2", "grass3", "grass4", "grass5", "grass6", "grass7", "grass8", "grass9", "grass10",
                          "pavement1", "pavement2", "pavement3", "pavement4", "pavement5", "pavement6", "pavement7", "pavement8", "pavement9", "pavement10",
                          "sand1", "sand2", "sand3", "sand4", "sand5", "sand6", "sand7", "sand8", "sand9", "sand10",
                          "snow1", "snow2", "snow3", "snow4", "snow5", "snow6", "snow7", "snow8", "snow9", "snow10",
                          "soil1", "soil2", "soil3", "soil4", "soil5", "soil6", "soil7", "soil8", "soil9", "soil10"]
    ground_list = ground_list_testing + ground_list_traing

    if do_training:
        # Phase 1: give the user some time for training
        for episode in range(start_episode_training, tot_episode_training):
            # Reset the ground every episode choosing randomly
            clean_world(ground_list)
            ground_index = STDrandom.randint(0, len(ground_list_traing) - 1)
            ground = ground_list_traing[ground_index]
            generate_new_world(ground, ground_list_traing)
            # send the UAV up in the air and set done=False
            send_action("takeoff")
            print ""
            print "Training episode:" + str(episode + 1)
            reset_pose()
            episode_start_timer = time.time()
            cumulated_reward = 0
            done = False
            # while (True): #TODO: substitude with while(!done)
            while not done:
                current_time = time.time()
                elapsed_time = current_time - episode_start_timer
                if (int(elapsed_time) % 2 == 0):
                    print "Remaining time: " + str(time_episode - elapsed_time)
                    done_reward = get_done_reward()
                    done = done_reward.done
                    reward = done_reward.reward
                    cumulated_reward += reward
                    print cumulated_reward
                    print ""
                    if elapsed_time > time_episode:
                        rospy.logwarn("Time elapsed!")
                        done = True
                        # break
                if done == True:
                    rospy.logwarn("Episode finished")
                    # break
                time.sleep(2.0)  # slow down cpu
            if done:
                if (cumulated_reward <= -1):
                    rospy.logerr("Reward: " + str(cumulated_reward))
                else:
                    rospy.logwarn("Reward: " + str(cumulated_reward))
        print "Training phase completed. Waiting for input before starting the test phase."
        raw_input()  # Wait for the user pressing a key button

    # Phase 2: now test the user on 7 ground classes

    # Reshape the groun_list list to remove bias due to tireness
    #np.random.choice(ground_list, size=len(ground_list), replace=False)

    for test_episode in range(start_episode_testing, tot_episode_testing):
        ground_index = test_episode / episode_per_ground
        if ground_index != actual_ground_index or (test_episode == start_episode_testing):
            if(test_episode < tot_episode_testing - 6):
                clean_world(ground_list)
                generate_new_world(ground_index, ground_list_testing)
            else:
                ground_index = len(ground_list) - 1
                if already_generated is False:
                    clean_world(ground_list)
                    generate_new_world_small(ground_list_testing_small)
                    already_generated = True
        actual_ground_index = ground_index
        send_action("takeoff")
        print ""
        print "Episode:" + str(test_episode + 1)
        reset_pose()
        rospy.sleep(2.0)
        episode_start_timer = time.time()
        print "Episode started!"
        cumulated_reward = 0
        step = 0
        done = False
        while not done:
            current_time = time.time()
            elapsed_time = current_time - episode_start_timer
            print "Remaining time: " + str(time_episode - elapsed_time)
            done_reward = get_done_reward()
            done = done_reward.done
            reward = done_reward.reward
            #get_relative_pose(ground_index, test_episode, step, reward)
            cumulated_reward += reward
            step += 1
            print cumulated_reward
            print ""
            if step >= tot_steps:
                print "Time elapsed!"
                done = True
                break
            if done == True:
                print "Episode finished"
                break
            rospy.sleep(2.0)  # slow down cpu
        if done:
            if (cumulated_reward <= -1):
                rospy.logerr("Reward: " + str(cumulated_reward))
            else:
                rospy.logwarn("Reward: " + str(cumulated_reward))
            total_reward += cumulated_reward
            print "######################################################"
            print "## Your cumulated reward is :" + str(total_reward) + "         ##"
            print "######################################################"
            with open("./results_" + str(datetime.date.today().strftime("%m%d%Y"))
                      + ".csv", "a") as myfile:
                boolean_reward = 0
                if(cumulated_reward > 0):
                    boolean_reward = 1
                ground_list = ["asphalt11", "asphalt12", "asphalt13",
                               "brick11", "brick12", "brick13",
                               "grass11", "grass12", "grass13",
                               "pavement11", "pavement12", "pavement13",
                               "sand11", "sand12", "sand13",
                               "snow11", "snow12", "snow13",
                               "soil11", "soil12", "soil13",
                               "mixed"]
                string_to_add = ground_list[ground_index] + "," + str(test_episode) + "," + str(
                    step) + "," + str(cumulated_reward) + "," + str(boolean_reward) + '\n'
                myfile.write(string_to_add)
        time.sleep(2.0)
    sys.exit(0)


if __name__ == "__main__":
    main()
