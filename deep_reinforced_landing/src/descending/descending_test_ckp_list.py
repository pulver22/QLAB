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


def get_image():
    """
    Get the last frame acquired by the camera.

    @return resp.image is the gresycale image acquired by the camera
    """

    # Access the global variable and activate the saving for the last camera's
    # frame
    global _save_image
    _save_image = True


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
    os.system("rosrun gazebo_ros spawn_model -file ~/.gazebo/models/" + model_to_add +
              "/model.sdf -sdf -model " + model_to_add + "_plane -x 0 -y 0 -z 0.0")


def get_random_action():
    """
    Choose a random action for the UAV.
    """
    # Define an array containing the available actions for the UAV
    # in the final work, takeoff and land must be added
    action_list = ['left', 'right', 'forward', 'backward', 'stop', 'descend']
    # Choose a random action within the array
    # action_index = STDrandom.randint(0, len(action_list) - 1)
    # forward,backward,left,right, stop and land
    probability_descend = 0.25
    probability = (1.0 - probability_descend) / (len(action_list) - 1)
    action_probability = [probability, probability, probability,
                          probability, probability, probability_descend]
    action = np.random.choice(action_list, 1, p=action_probability)[0]
    # action_index = STDrandom.randint(0, 10)
    # action = action_list[action_index]

    return action


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


def convert_action_batch_str_to_int(action_t_batch):
    """
    Convert a batch containing actions expressed as string in integer
    (as required by the gradient descent method).

    @param action_batch is the array contatining actions as string
    @return action_batch is the array containing actions as integer
    """

    # action_list = ['left', 'right', 'forward', 'backward', 'stop', 'land',
    # 'left_forward', 'left_backward', 'right_forward', 'right_backward'
    # ,'descend', 'ascend', 'rotate_left', 'rotate_right']
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
    # action_list = ['left', 'right', 'forward', 'backward', 'stop', 'land', 'left_forward', 'left_backward', 'right_forward', 'right_backward' ,'descend', 'ascend', 'rotate_left', 'rotate_right']
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


def get_relative_pose(ground_index, episode, step, reward, ground_list, ckp):
    """
    Get the relative pose of the quadrotor to the marker.
    @return a log containing information such as the ground used, episode, step, reward and the relative pose of the quadrotor
    """
    rospy.wait_for_service('/drl/get_relative_pose')
    try:
        get_relative_pose_proxy = rospy.ServiceProxy(
            '/drl/get_relative_pose', GetRelativePose)
        resp = get_relative_pose_proxy()

        with open("./" + str(ckp) + "/relative_distances_ckp" + str(ckp) + "_" + str(datetime.date.today().strftime("%m%d%Y")) + ".csv", "a") as myfile:

            string_to_add = ground_list[ground_index] + "," + str(episode) + "," + str(step) + "," + str(reward) + "," + str(
                resp.pose.position.x) + "," + str(resp.pose.position.y) + "," + str(resp.pose.position.z) + '\n'
            myfile.write(string_to_add)
    except rospy.ServiceException, ex:
        print "Service call get_relative_pose failed: %e" % ex


def main():
    """
    Main function for training the DQN network in learning how to accomplish autonomous landing.
    """

    # ground_list = ["water1",
    #             "water2",
    #             "water3",
    #             "water4",
    #             "water5",
    #             "water6",
    #             "water7",
    #             "water8",
    #             "water9",
    #             "water10"]

    # ground_list = ["water11", "water12", "water13",
    #    "water14", "water15", "water16",
    #    "water17", "water18", "water19"]

    ground_list = ["water12", "water15", "water17"]
    ckp_list = ["92250", "93000", "93750", "94500", "95250"]
    # ATTENTION: If you want to restore files from a previous simulation you
    # must pass valid values for these variables:
    # policy_weights_path =
    # '/home/pulver/Desktop/marine_descending/checkpoint/episode_94500/policy/policy_checkpoint.ckp'
    summary_folder = ""  # if empty the summary is written in ./log/ + current time
    start_episode = 0  # change to last episode number
    frame_counter = 0  # change to last number of frames

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

    num_ground_plane = len(ground_list)
    actual_ground_index = 0
    # episode_per_ground specify the number of episodes with the same ground
    # plane
    episode_per_ground = 100
    tot_episodes = episode_per_ground * num_ground_plane
    accuracy_ground = 0.0

    timer_total_start = time.time()
    rospy.init_node("DRLTestNode")
    rospy.loginfo("----- DRL Test Node -----")

    # Create a subscriber fot the greyscale image
    # rospy.Subscriber("/drl/grey_camera", ROSImage, image_callback)#TODO
    # restore default
    rospy.Subscriber(
        "/quadrotor/ardrone/bottom/ardrone/bottom/image_raw", ROSImage, image_callback, queue_size=30)  # Store the last 30 messages before discarding them

    images_stack_size = 4

    # r = rospy.Rate(10)  # 10hz

    # Init session and networks
    sess = tf.Session()
    policy_network = QNetwork(sess, tot_actions=tot_actions, image_shape=(
        screen_width, screen_height, images_stack_size), batch_size=batch_size, network_name="policy_net")
    init = tf.global_variables_initializer()
    sess.run(init)

    for i in range(len(ckp_list)):
        ckp = ckp_list[i]
        policy_weights_path = '/home/pulver/Desktop/marine_descending/checkpoint/episode_' + \
            str(ckp) + '/policy/policy_checkpoint.ckp'

        if(summary_folder == ""):
            tf_summary_writer = tf.summary.FileWriter(
                './' + str(ckp) + '/log/' + str(datetime.datetime.now().time()), sess.graph)
        else:
            tf_summary_writer = tf.summary.FileWriter(
                summary_folder, sess.graph)

        if(policy_weights_path != ""):
            print("Loading weights from memory: " + str(policy_weights_path))
            policy_network.load_weights(policy_weights_path)
        else:
            print("The networks path are empty. Choose appropriate weights!")
            sys.exit()

        # start here
        for episode in range(start_episode, tot_episodes):
            # Reset the ground in the environment every episode_per_ground
            ground_index = episode / episode_per_ground
            print "Current ground index: " + str(ground_index)
            if episode % (episode_per_ground) == 0 and episode != start_episode:
                with open("./" + str(ckp) + "/accuracy_ckp" + str(ckp) + ".csv", "a") as myfile:
                    accuracy_ground = accuracy_ground / \
                        (episode_per_ground * 1.0)
                    print "Final accuracy value: " + str(accuracy_ground)
                    string_to_add = ground_list[ground_index - 1] + \
                        "," + str(accuracy_ground) + "\n"
                    myfile.write(string_to_add)

            if ground_index != actual_ground_index or (episode == start_episode):
                accuracy_ground = 0.0
                clean_world(ground_list)
                generate_new_world(ground_index, ground_list)
            actual_ground_index = ground_index
            timer_start = time.time()
            actual_time = rospy.get_rostime()
            rospy_start_time = actual_time.secs + actual_time.nsecs / 1000000000.0
            frame_episode = 0
            cumulated_reward = 0
            epsilon_used = 0
            send_action("takeoff")
            print("")
            print("Episode: " + str(episode))
            # 0-Reset USV's position
            os.system(
                "rosservice call /gazebo/set_model_state '{model_state: { model_name: usv_model, pose: { position: { x: 0.0, y: 0.0 ,z: 5.0 }, orientation: {x: 0, y: 0.0, z: 0, w: 0.0 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'")
            # 1-Accumulate the first state
            reset_pose()
            print "Reset pose!"
            send_action("stop")
            rospy.sleep(1.0)
    #        get_image()
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
                done_reward = get_done_reward()
                reward = done_reward.reward
                done = done_reward.done
                image_t1 = np.expand_dims(image_t1, 2)
                # stack the images
                image_t1 = np.append(image_t[:, :, 1:], image_t1, axis=2)
                frame_counter += 1  # To call every time a frame is obtained
                frame_episode += 1
                image_t = image_t1
                get_relative_pose(ground_index, episode,
                                  step, reward, ground_list, ckp)
                cumulated_reward += reward
                # At every step check if more than 30 seconds from the start passed.
                # In that case, set done = True and end the episode
                timer_stop = time.time()
                # Stop the episode if the number of frame is more than a
                # threshold
                if frame_episode >= steps_per_episodes:
                    done = True
                # When the episode is done
                if done:
                    timer_stop = time.time()
                    actual_time = rospy.get_rostime()
                    rospy_stop_time = actual_time.secs + actual_time.nsecs / 1000000000.0
                    rospy_time_elapsed = rospy_stop_time - rospy_start_time
                    print("Tot Frame counter: " + str(frame_counter))
                    print("Time episode: " +
                          str(timer_stop - timer_start) + " seconds")
                    print("Ros time episode: " +
                          str(rospy_time_elapsed) + " seconds")
                    if cumulated_reward >= 0:
                        rospy.logwarn("Positive reward obtained!")
                    print("Cumulated reward: " + str(cumulated_reward))
                    print("Episode finished after {} timesteps".format(step + 1))
                    sys.stdout.flush()

                    with open("./" + str(ckp) + "/results_ckp" + str(ckp) + "_" + str(datetime.date.today().strftime("%m%d%Y"))
                              + ".csv", "a") as myfile:
                        boolean_reward = 0
                        if(cumulated_reward > 0):
                            boolean_reward = 1
                            accuracy_ground += 1.0
                        print "Current accuracy value: " + str(accuracy_ground)
                        string_to_add = ground_list[ground_index] + "," + str(episode) + "," + str(
                            step) + "," + str(cumulated_reward) + "," + str(boolean_reward) + '\n'
                        myfile.write(string_to_add)

                    if episode == tot_episodes - 1:
                        with open("./" + str(ckp) + "/accuracy_ckp" + str(ckp) + ".csv", "a") as myfile:
                            accuracy_ground = accuracy_ground / \
                                (episode_per_ground * 1.0)
                            print "Final accuracy value: " + str(accuracy_ground)
                            string_to_add = ground_list[actual_ground_index] + \
                                "," + str(accuracy_ground) + "\n"
                            myfile.write(string_to_add)

                    break


if __name__ == "__main__":
    main()
