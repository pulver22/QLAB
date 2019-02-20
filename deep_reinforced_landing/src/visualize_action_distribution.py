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


def generate_new_world(model_to_add):
    """
    Remove the old model on the floor and add new ones.

    @param model_to_remove is the name of the model to remove (all its istances)
    @param model_to_add is the name of the model to add
    """
    #  First remove all the instance of the old model
    #  25 is the maximum number of istances of the model in the world
    ground_list = ["asphalt11","asphalt12","asphalt13", 
            "brick11", "brick12", "brick13", 
            "grass11", "grass12", "grass13", 
            "pavement11", "pavement12", "pavement13", 
            "sand11", "sand12", "sand13", 
            "snow11", "snow12", "snow13", 
            "soil11", "soil12", "soil13"]

    for ground in ground_list:
        try:
            ground = ground + "_plane"
            remove_model(ground)
        except:
            pass

    if type(model_to_add) is int:
        # print "Is an int!"
        model_to_add = ground_list[model_to_add]
        print "Ground choosen is " + str(model_to_add)

    # Spawn new istances for the new model
    os.system("rosrun gazebo_ros spawn_model -file ~/.gazebo/models/" +
              model_to_add + "/model.sdf -sdf -model " + model_to_add + "_plane -x 0 -y 0")


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


def choose_random_ground():
    """
    Return the name of the new ground to select
    """

    ground_list = ["asphalt11","asphalt12","asphalt13", 
            "brick11", "brick12", "brick13", 
            "grass11", "grass12", "grass13", 
            "pavement11", "pavement12", "pavement13", 
            "sand11", "sand12", "sand13", 
            "snow11", "snow12", "snow13", 
            "soil11", "soil12", "soil13"]
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


def generate_new_world(model_to_add):
    """
    Remove the old model on the floor and add new ones.

    @param model_to_remove is the name of the model to remove (all its istances)
    @param model_to_add is the name of the model to add
    """
    #  First remove all the instance of the old model
    #  25 is the maximum number of istances of the model in the world
    #ground_list = ["asphalt_plane", "concretefloors_plane", "farmland_plane", "floorsregular_plane", 
    #             "grassdead_plane", "gravelcobble_plane", "groundplants_plane", "roadmarkings_plane", 
    #             "roads1_plane", "roads2_plane", "roads3_plane", "roads4_plane", 
    #             "sandpabbles1_plane", "sandpabbles2_plane", "snow_plane", "soilsand_plane"]
    
    ground_list = ["asphalt11","asphalt12","asphalt13", 
            "brick11", "brick12", "brick13", 
            "grass11", "grass12", "grass13", 
            "pavement11", "pavement12", "pavement13", 
            "sand11", "sand12", "sand13", 
            "snow11", "snow12", "snow13", 
            "soil11", "soil12", "soil13"]

    for ground in ground_list:
        try:
            ground = ground + "_plane"
            remove_model(ground)            
        except:
            pass
    
    if type(model_to_add) is int:
        #print "Is an int!"
        model_to_add = ground_list[model_to_add] 
        print "Ground choosen is " + str(model_to_add)
        
    # Spawn new istances for the new model
    os.system("rosrun gazebo_ros spawn_model -file ~/.gazebo/models/" + model_to_add +"/model.sdf -sdf -model " + model_to_add + "_plane -x 0 -y 0")

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
    policy_weights_path = '/home/pulver/Desktop/episode_32400/policy/policy_checkpoint.ckp'

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
    tot_episodes = 1050
    steps_per_episodes = 20  # expressed in number step
    noop_time = 2.0  # pause in seconds between actions

    num_ground_plane = 21
    episodes_per_ground_plane = tot_episodes / num_ground_plane
    actual_ground_index = 0

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

    print "Episode, step, action, reward, action_distribution"
    # start here
    for episode in range(start_episode, tot_episodes):
        # Reset the ground in the environment every 25k frames
        # ground_index = episode / 50
        # if ground_index != actual_ground_index or episode == 0:
        #     generate_new_world(ground_index)
        # actual_ground_index = ground_index
        timer_start = time.time()
        actual_time = rospy.get_rostime()
        rospy_start_time = actual_time.secs + actual_time.nsecs / 1000000000.0
        cumulated_reward = 0   
        # 1-Accumulate the first state
        #reset_pose()
        send_action("stop")
        rospy.sleep(1.0)
        state = _last_image
        # create a stack of X images
        image_t = np.stack([state] * images_stack_size, axis=2)
        frame_episode = 0
        pad_size = 3
        pad_value = 255
        for step in range(tot_steps):
            # 2- Get the action following epsilon-greedy or through policy network.
            # With probability epsilon take random action otherwise it takes
            # an action from the policy network.
            # Take a random action
            
            image_stack_padded = [np.lib.pad(image_t[:,:,0], (pad_size, pad_size), 'constant', constant_values=(pad_value, pad_value)),
            np.lib.pad(image_t[:,:,1], (pad_size, pad_size), 'constant', constant_values=(pad_value, pad_value)),
            np.lib.pad(image_t[:,:,2], (pad_size, pad_size), 'constant', constant_values=(pad_value, pad_value)),
            np.lib.pad(image_t[:,:,3], (pad_size, pad_size), 'constant', constant_values=(pad_value, pad_value))]
            image_hstack = np.hstack(image_stack_padded)
            cv2.imwrite("/home/pulver/Desktop/print_network_action_distribution/img_ep"+str(episode)+"_step" + str(step) + ".png", image_hstack )            
            # Here image_t is always ready and it can be given as input to
            # the network
            action_distribution = policy_network.return_action_distribution( input_data=np.reshape(image_t, (1, 84, 84, images_stack_size)), softmax=False)
            
            action = np.argmax(action_distribution)
            action = convert_action_int_to_str(action)
            #action = get_random_action()
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
            #get_relative_pose(ground_index, episode, step, reward)
            # At every step check if more than 30 seconds from the start passed.
            # In that case, set done = True and end the episode
            timer_stop = time.time()
            print str(episode) + "," + str(step) + "," + action + "," + str(reward) + "," +str(action_distribution)
            # Stop the episode if the number of frame is more than a threshold
            if frame_episode >= steps_per_episodes:
                done = True
            # if timer_stop - timer_start >= 30.0: # maximum time allowed
            #     #cumulated_reward += -1
            #     done = True
            # When the episode is done
            if done:
                break


if __name__ == "__main__":
    main()
