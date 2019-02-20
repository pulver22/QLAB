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
import tensorflow as tf
import numpy as np
import sys
from cv_bridge import CvBridge, CvBridgeError
from q_network import QNetwork
from experience_replay_buffer import ExperienceReplayBuffer
import Image
import datetime
import time
import os.path
import cv2
# Adding these two lines solved the crash of Tesla K40
import os
os.environ["CUDA_VISIBLE_DEVICES"] = "0"
from deep_reinforced_landing.srv import NewCameraService, GetDoneAndReward, SendCommand, ResetPosition, GetRelativePose  # DRL services
from gazebo_msgs.srv import DeleteModel  # Gazebo service for removing a model
import rospy
# Rename to avoid confusion with Image lib
from sensor_msgs.msg import Image as ROSImage
import subprocess  # needed for using bash command


DEBUG = False  # Set to False to disable the image shown at the begining
_last_image = None  # Global variable representing the last frame acquired by the camera
#_save_image = False


def image_callback(img_msg):
    """
    When a new image is published, save the last frame in a global variable
    """
    bridge = CvBridge()
    try:
        # Convert from sensor_msgs::Image to cv::Mat
        cv_image = bridge.imgmsg_to_cv2(
            img_msg, desired_encoding="passthrough")
        # Access global variable and store image as numpy.array
        global _last_image
        _last_image = np.asarray(cv_image)
    except CvBridgeError as ex:
        print "ERROR!!"
        print ex


# def get_image():
#     """
#     Get the last frame acquired by the camera.

#     @return resp.image is the gresycale image acquired by the camera
#     """

#     # Access the global variable and activate the saving for the last camera's
#     # frame
#     global _save_image
#     _save_image = True


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
        print "Action sent: " + str(action)
    except rospy.ServiceException, ex:
        print "Service call get_random_action failed: %s" % ex


def convert_action_batch_str_to_int(action_t_batch):
    """
    Convert a batch containing actions expressed as string in integer
    (as required by the gradient descent method).

    @param action_batch is the array contatining actions as string
    @return action_batch is the array containing actions as integer
    """

    #action_list = ['left', 'right', 'forward', 'backward', 'stop', 'land', 'left_forward', 'left_backward', 'right_forward', 'right_backward' ,'descend', 'ascend', 'rotate_left', 'rotate_right']
    for i in range(len(action_t_batch)):
        if action_t_batch[i] == 'left':
            action_t_batch[i] = 0
        elif action_t_batch[i] == 'right':
            action_t_batch[i] = 1
        elif action_t_batch[i] == 'forward':
            action_t_batch[i] = 2
        elif action_t_batch[i] == 'backward':
            action_t_batch[i] = 3
        elif action_t_batch[i] == 'stop':
            action_t_batch[i] = 4
        elif action_t_batch[i] == 'descend':
            action_t_batch[i] = 5
    return action_t_batch


def convert_action_int_to_str(action):
    """
    Convert an action expressed as integer into its string value.

    @param action is an integer representing which action UAV has to take
    @return action as string value
    """
    #action_list = ['left', 'right', 'forward', 'backward', 'stop', 'land', 'left_forward', 'left_backward', 'right_forward', 'right_backward' ,'descend', 'ascend', 'rotate_left', 'rotate_right']
    # Conversion from integer to string
    if action == 0:
        action = 'left'
    elif action == 1:
        action = 'right'
    elif action == 2:
        action = 'forward'
    elif action == 3:
        action = 'backward'
    elif action == 4:
        action = 'stop'
    elif action == 5:
        action = 'descend'
    return action


def main():
    """
    Main function for training the DQN network in learning how to accomplish autonomous landing.
    """
    # ATTENTION: If you want to restore files from a previous simulation you
    # must pass valid values for these variables:
    policy_weights_path = '/home/pulver/Desktop/episode_113250/policy/policy_checkpoint.ckp'
    summary_folder = ""  # if empty the summary is written in ./log/ + current time

    screen_width = 84  # original is 160
    screen_height = 84  # original is 210
    images_stack_size = 4
    # Use only the first 5 actions for this simulation
    # action_list = ['left', 'right', 'forward', 'backward', 'stop', 'descend']
    # 0 (left, 1 (right), 2 (forward), 3 (backward), 4 (stop), 5 (descend)
    tot_actions = 6
    batch_size = 32  # size of the experience batch
    tot_steps = 1000  # finite-horizont simulation
    # 10x10^6 high number since training can be stopped at every time

    steps_per_episodes = 40  # expressed in number step
    noop_time = 2.0  # pause in seconds between actions

    timer_total_start = time.time()
    rospy.init_node("DRLTestNode")
    rospy.loginfo("----- DRL Test Node -----")

    # Create a subscriber fot the greyscale image
    rospy.Subscriber("/drl/grey_camera", ROSImage, image_callback)  # TODO
    # restore default
    # rospy.Subscriber("/quadrotor/ardrone/bottom/ardrone/bottom/image_raw", ROSImage, image_callback, queue_size=30)  # Store the last 30 messages
    # before discarding them

    images_stack_size = 4

    r = rospy.Rate(10)  # 10hz

    # Init session and networks
    sess = tf.Session()
    if(summary_folder == ""):
        tf_summary_writer = tf.summary.FileWriter(
            './log/' + str(datetime.datetime.now().time()), sess.graph)
    else:
        tf_summary_writer = tf.summary.FileWriter(summary_folder, sess.graph)
    policy_network = QNetwork(sess, tot_actions=tot_actions, image_shape=(
        screen_width, screen_height, images_stack_size), batch_size=batch_size, network_name="policy_net")

    init = tf.global_variables_initializer()
    sess.run(init)

    if(policy_weights_path != ""):
        print("Loading weights from memory: " + str(policy_weights_path))
        policy_network.load_weights(policy_weights_path)
    else:
        print("The networks path are empty. Choose appropriate weights!")
        sys.exit()

    ##########################################################################
    ##                                START HERE                              ##
    ##########################################################################
    timer_start = time.time()
    actual_time = rospy.get_rostime()
    rospy_start_time = actual_time.secs + actual_time.nsecs / 1000000000.0
    frame_episode = 0
    cumulated_reward = 0
    send_action("takeoff")
    # 1-Accumulate the first state
    send_action("stop")
    rospy.sleep(1.0)
    # acquire image from bottomcamera
    # get_image()
    state = _last_image
    # create a stack of X images
    image_t = np.stack([state] * images_stack_size, axis=2)

    for step in range(tot_steps):
        # 2- Get the action following epsilon-greedy or through policy network.
        # Here image_t is always ready and it can be given as input to
        # the network
        action_distribution = policy_network.return_action_distribution(
            input_data=np.reshape(image_t, (1, 84, 84, images_stack_size)), softmax=False)
        # print ""
        # print action_distribution
        action = np.argmax(action_distribution)
        action = convert_action_int_to_str(action)
        # print action
        send_action(action)
        rospy.sleep(noop_time)
        # 3- Get the state_t1 repeating the action obtained at step-2 and add everything
        # in the replay buffer. Then set state_t = state_t1
        # get_image()
        image_t1 = _last_image
        send_action("stop")
        # If the action taken is to land and the UAV is inside the landing
        # BB, done will be calculated accordingly

        # NOTE: in the real implementation there's no way to get done and reward
        # done_reward = get_done_reward()
        # reward = done_reward.reward
        # done = done_reward.done

        image_t1 = np.expand_dims(image_t1, 2)
        # stack the images
        image_t1 = np.append(image_t[:, :, 1:], image_t1, axis=2)
        frame_episode += 1
        image_t = image_t1

        # cumulated_reward += reward

        # At every step check if more than 30 seconds from the start passed.
        # In that case, set done = True and end the episode
        timer_stop = time.time()
        # Stop the episode if the number of frame is more than a threshold
        if frame_episode >= steps_per_episodes:
            done = True
        # When the episode is done
        if done:
            timer_stop = time.time()
            actual_time = rospy.get_rostime()
            rospy_stop_time = actual_time.secs + actual_time.nsecs / 1000000000.0
            rospy_time_elapsed = rospy_stop_time - rospy_start_time
            print("Time episode: " + str(timer_stop - timer_start) + " seconds")
            print("Ros time episode: " +
                  str(rospy_time_elapsed) + " seconds")
            # if cumulated_reward >= 0:
            #     rospy.logwarn("Positive reward obtained!")
            # print("Cumulated reward: " + str(cumulated_reward))
            print("Episode finished after {} timesteps".format(step + 1))
            sys.stdout.flush()
            break


if __name__ == "__main__":
    main()
