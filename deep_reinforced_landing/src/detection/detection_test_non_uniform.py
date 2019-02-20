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
from deep_reinforced_landing.srv import NewCameraService, GetDoneAndReward, SendCommand, ResetPosition, GetRelativePose
from gazebo_msgs.srv import DeleteModel
import rospy
# Rename to avoid confusion with Image lib
from sensor_msgs.msg import Image as ROSImage
import subprocess  # needed for using bash command

GAZEBO_MODEL_PATH = "~/.gazebo/models/"


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
            ground_list = ["asphalt11","asphalt12","asphalt13", 
            "brick11", "brick12", "brick13", 
            "grass11", "grass12", "grass13", 
            "pavement11", "pavement12", "pavement13", 
            "sand11", "sand12", "sand13", 
            "snow11", "snow12", "snow13", 
            "soil11", "soil12", "soil13"]

            string_to_add = ground_list[ground_index] + "," + str(episode) + "," + str(step) + "," + str(reward) + "," + str(resp.pose.position.x) + "," + str(resp.pose.position.y) + "," + str(resp.pose.position.z) + '\n'
            myfile.write(string_to_add)
    except rospy.ServiceException, ex:
        print "Service call get_relative_pose failed: %e" % ex


def clean_world(ground_list):
    for i in range(25):
        try:
            remove_model(str(i) + "_plane")
        except:
            pass


def generate_new_world(ground_list):
    """
    Remove the old model on the floor and add new ones.

    @param model_to_remove is the name of the model to remove (all its istances)
    @param model_to_add is the name of the model to add
    """

    # model_to_add = []
    # for i in range(0, 50):
    #     index = STDrandom.randint(0, len(ground_list) - 1)
    #     model_to_add.append(ground_list[index])
    ground_list = ["asphalt11_small", "asphalt12_small", "asphalt13_small",
                   "brick11_small", "brick12_small", "brick13_small",
                   "grass11_small", "grass12_small", "grass13_small",
                   "pavement11_small", "pavement12_small", "pavement13_small",
                   "sand11_small", "sand12_small", "sand13_small",
                   "snow11_small", "snow12_small", "snow13_small",
                   "soil11_small", "soil12_small", "soil13_small"]
    model_to_add_array = np.random.randint(0, len(ground_list), 25)


    print "\n"
    # rospy.logwarn("Ground choosen is " + str(model_to_add_array))
    # Spawn new istances for the new model
    #os.system("rosrun gazebo_ros spawn_model -file ~/.gazebo/models/" + model_to_add +"/model.sdf -sdf -model " + model_to_add + "_plane -x 0 -y 0")
    coord_xy_list = [(a, b) for a in range(-20, 30, 10)
                     for b in range(-20, 30, 10)]

    for i, coord_xy in enumerate(coord_xy_list):
        print str(model_to_add_array[i])
        os.system("rosrun gazebo_ros spawn_model -file ~/.gazebo/models/" +
                  ground_list[model_to_add_array[i]] + "/model.sdf -sdf -model " + str(i) + "_plane -x " + str(coord_xy[0]) + " -y " + str(coord_xy[1]))

    
    print "Waiting 5 sec"
    time.sleep(10)
    print "Waiting done!"
    


def get_random_action():
    """
    Choose a random action for the UAV.
    """
    # Define an array containing the available actions for the UAV
    # in the final work, takeoff and land must be added
    action_list = ['left', 'right', 'forward', 'backward', 'stop', 'land',
                   'descend', 'ascend', 'rotate_left', 'rotate_right']
    # Choose a random action within the array
    #action_index = STDrandom.randint(0, len(action_list) - 1)
    # forward,backward,left,right, stop and land
    action_index = STDrandom.randint(0, 5)
    action = action_list[action_index]

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


def convert_action_batch_str_to_int(action_t_batch):
    """
    Convert a batch containing actions expressed as string in integer
    (as required by the gradient descent method).

    @param action_batch is the array contatining actions as string
    @return action_batch is the array containing actions as integer
    """
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
        elif action_t_batch[i] == 'land':
            action_t_batch[i] = 5
        elif action_t_batch[i] == 'descend':
            action_t_batch[i] = 6
        elif action_t_batch[i] == 'ascend':
            action_t_batch[i] = 7
        elif action_t_batch[i] == 'rotate_left':
            action_t_batch[i] = 8
        elif action_t_batch[i] == 'rotate_right':
            action_t_batch[i] = 9

    return action_t_batch


def convert_action_int_to_str(action):
    """
    Convert an action expressed as integer into its string value.

    @param action is an integer representing which action UAV has to take
    @return action as string value
    """
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
        action = 'land'
    elif action == 6:
        action = 'descend'
    elif action == 7:
        action = 'ascend'
    elif action == 8:
        action = 'rotate_left'
    elif action == 9:
        action = 'rotate_right'

    return action


def main():
    """
    Main function for training the DQN network in learning how to accomplish autonomous landing.
    """
    # ATTENTION: If you want to restore files from a previous simulation you
    # must pass valid values for these variables:
    policy_weights_path = '/home/pulver/Desktop/DQN-single/episode_36000/policy/policy_checkpoint.ckp'

    ground_list = ["asphalt11_small", "asphalt12_small", "asphalt13_small",
                   "brick11_small", "brick12_small", "brick13_small",
                   "grass11_small", "grass12_small", "grass13_small",
                   "pavement11_small", "pavement12_small", "pavement13_small",
                   "sand11_small", "sand12_small", "sand13_small",
                   "snow11_small", "snow12_small", "snow13_small",
                   "soil11_small", "soil12_small", "soil13_small"]

    summary_folder = ""  # if empty the summary is written in ./log/ + current time
    start_episode = 0  # change to last episode number
    frame_counter = 0  # change to last number of frames

    screen_width = 84  # original is 160
    screen_height = 84  # original is 210
    images_stack_size = 4
    # Use only the first 5 actions for this simulation
    # 0 (left, 1 (right), 2 (forward), 3 (backward), 4 (stop), 5 (land),
    # 6 (ascend), 7 (descend), 8 (rotate_left), 9 (rotate_right)
    tot_actions = 6
    batch_size = 32  # size of the experience batch
    tot_steps = 1000  # finite-horizont simulation
    # 10x10^6 high number since training can be stopped at every time
    tot_episodes = 50
    steps_per_episodes = 20  # expressed in number step
    noop_time = 2.0  # pause in seconds between actions

    num_ground_plane = 21
    actual_ground_index = 0
    # episode_per_ground specify the number of episodes with the same ground
    # plane
    ground_counter = 1
    episode_per_ground = 10

    timer_total_start = time.time()
    rospy.init_node("DeepReinforcedLanding")
    rospy.loginfo("----- Deep Reinforced Landing Node -----")

    rospy.Subscriber(
        "/quadrotor/ardrone/bottom/ardrone/bottom/image_raw", ROSImage, image_callback, queue_size=30)  # Store the last 30 messages before discarding them

    r = rospy.Rate(10)  # 10hz

    # Init session and networks
    sess = tf.Session()
    if(summary_folder == ""):
        tf_summary_writer = tf.summary.FileWriter(
            './log/' + str(datetime.datetime.now().time()), sess.graph)
    else:
        tf_summary_writer = tf.summary.FileWriter(
            summary_folder, sess.graph)
    policy_network = QNetwork(sess, tot_actions=tot_actions, image_shape=(
        screen_width, screen_height, images_stack_size), batch_size=batch_size, network_name="policy_net")
    init = tf.global_variables_initializer()
    sess.run(init)

    # Load Neural Networks weights from memory if a valid checkpoint path is passed
    # if(os.path.isfile(policy_weights_path) == True and
    # os.path.isfile(target_weights_path) == True):
    try:
        print("Loading weights from memory...")
        policy_network.load_weights(policy_weights_path)
    except:
        print("Error while loading the weights!")
        return

    # start here
    for episode in range(start_episode, tot_episodes):
        # Reset the ground in the environment every 50 episodes (or
        # episode_per_ground)
        ground_index = episode / episode_per_ground
        if ground_index != actual_ground_index or (episode == start_episode):
            clean_world(ground_list)
            generate_new_world(ground_list)
        actual_ground_index = ground_index

        timer_start = time.time()
        actual_time = rospy.get_rostime()
        rospy_start_time = actual_time.secs + actual_time.nsecs / 1000000000.0
        cumulated_reward = 0
        print("")
        print("Episode: " + str(episode))
        # 1-Accumulate the first state
        reset_pose()
        print "Reset pose!"
        send_action("stop")
        rospy.sleep(1.0)
        state = _last_image
        # create a stack of X images
        image_t = np.stack([state] * images_stack_size, axis=2)
        frame_episode = 0
        for step in range(tot_steps):
            # 2- Get the action following epsilon-greedy or through policy network.
            # With probability epsilon take random action otherwise it takes
            # an action from the policy network.
            # Take a random action

            # Here image_t is always ready and it can be given as input to
            # the network
            action_distribution = policy_network.return_action_distribution( input_data=np.reshape(image_t, (1, 84, 84, images_stack_size)), softmax=False)
            action = np.argmax(action_distribution)
            action = convert_action_int_to_str(action)
            # action = get_random_action()
            send_action(action)
            rospy.sleep(noop_time)
            # 3- Get the state_t1 repeating the action obtained at step-2 and add everything
            # in the replay buffer. Then set state_t = state_t1
            # get_image()
            image_t1 = _last_image
            # If the action taken is to land and the UAV is inside the landing
            # BB, done will be calculated accordingly
            done_reward = get_done_reward()
            reward = done_reward.reward
            done = done_reward.done
            image_t1 = np.expand_dims(image_t1, 2)
            # stack the images
            image_t1 = np.append(image_t[:, :, 1:], image_t1, axis=2)
            image_t = image_t1
            cumulated_reward += reward
            send_action("stop")
            frame_episode = frame_episode + 1

            # get_relative_pose(ground_index, episode, step, reward)

            # At every step check if more than 30 seconds from the start passed.
            # In that case, set done = True and end the episode
            timer_stop = time.time()
            # Stop the episode if the number of frame is more than a threshold
            if frame_episode >= steps_per_episodes:
                done = True
            # if timer_stop - timer_start >= 30.0: # maximum time allowed
            #     #cumulated_reward += -1
            #     done = True
            # When the episode is done
            if done:
                timer_stop = time.time()
                actual_time = rospy.get_rostime()
                rospy_stop_time = actual_time.secs + actual_time.nsecs / 1000000000.0
                rospy_time_elapsed = rospy_stop_time - rospy_start_time
                print("Tot Frame counter: " + str(frame_counter))
                print("Time episode: " + str(timer_stop - timer_start) + " seconds")
                print("Ros time episode: " +
                      str(rospy_time_elapsed) + " seconds")
                if cumulated_reward >= 0:
                    rospy.logwarn("Positive reward obtained!")
                print("Cumulated reward: " + str(cumulated_reward))
                print("Episode finished after {} timesteps".format(step + 1))
                sys.stdout.flush()
                # time.sleep(5)
                with open("./results_" + str(datetime.date.today().strftime("%m%d%Y")) 
                      + ".csv", "a") as myfile:
                    boolean_reward = 0
                    if(cumulated_reward > 0):
                        boolean_reward = 1
                    ground_list = "mixed"
                    string_to_add = "mixed" + "," + str(episode) + "," + str(step) + "," + str(cumulated_reward) + "," + str(boolean_reward) + '\n'
                    myfile.write(string_to_add)
                    break
                break


if __name__ == "__main__":
    main()
