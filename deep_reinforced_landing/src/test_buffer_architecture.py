#!/usr/bin/env python

# The MIT License (MIT)
# Copyright (c) 2017 Riccardo Polvara
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# Create, fill and store a Experience Replay FIFO buffer in tensorflow.
# An experience is defined by the current state (a sequence of images acquired with the camea),
# the action take in this state, the reward and the next state
import random as STDrandom
import time
import rospy
import numpy as np
import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import StringIO
import os
from ardrone_autonomy.msg import Navdata
# Rename to avoid confusion with Image lib
from sensor_msgs.msg import Image as ROSImage
from experience_replay_buffer import ExperienceReplayBuffer
from deep_reinforced_landing.srv import NewCameraService, GetDoneAndReward, SendCommand, ResetPosition
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

GAZEBO_MODEL_PATH = "~/.gazebo/models/"

DEBUG = False  # Set to False to disable the image shown at the begining
_last_image = None  # Global variable representing the last frame acquired by the camera
_save_image = False


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
        print ex


def get_image():
    """
    Get the last frame acquired by the camera.

    @return resp.image is the 84x84 gresycale image acquired by the camera
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


def get_random_action():
    """
    Choose a random action for the UAV.
    """
    # Define an array containing the available actions for the UAV
    # in the final work, takeoff and land must be added
    action_list = ['left', 'right', 'forward', 'backward', 'stop', 'descend']
    # Choose a random action within the array
    #action_index = STDrandom.randint(0, len(action_list) - 1)
    # forward,backward,left,right, stop and land
    probability_descend = 0.25
    probability = (1 - probability_descend) / (len(action_list) - 1)
    action_probability = [probability, probability, probability,
                          probability, probability, probability_descend]
    action = np.random.choice(action_list, 1, p=action_probability)[0]
    #action_index = STDrandom.randint(0, 10)
    #action = action_list[action_index]

    return action


def choose_random_ground():
    """
    Return the name of the new ground to select
    """

    ground_list = ["asphalt1", "asphalt2", "asphalt3", "asphalt4", "asphalt5", "asphalt6", "asphalt7", "asphalt8", "asphalt9", "asphalt10",
                   "brick1", "brick2", "brick3", "brick4", "brick5", "brick6", "brick7", "brick8", "brick9", "brick10",
                   "grass1", "grass2", "grass3", "grass4", "grass5", "grass6", "grass7", "grass8", "grass9", "grass10",
                   "pavement1", "pavement2", "pavement3", "pavement4", "pavement5", "pavement6", "pavement7", "pavement8", "pavement9", "pavement10",
                   "sand1", "sand2", "sand3", "sand4", "sand5", "sand6", "sand7", "sand8", "sand9", "sand10",
                   "snow1", "snow2", "snow3", "snow4", "snow5", "snow6", "snow7", "snow8", "snow9", "snow10",
                   "soil1", "soil2", "soil3", "soil4", "soil5", "soil6", "soil7", "soil8", "soil9", "soil10"]

    ground_index = STDrandom.randint(0, len(ground_list) - 1)
    ground = ground_list[ground_index]
    return ground


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


def set_pose(new_pose):
    """
    Reset the UAV's randomly inside the flight Bounding Box.
    """
    rospy.wait_for_service('/drl/set_model_state')
    try:
        reset_pose_proxy = rospy.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)
        reset_pose_proxy(new_pose)
    except rospy.ServiceException, ex:
        print "Service call set_pose failed: %s" % ex


def remove_model(model):
    """
    Remove the model from the world. 

    @param model is the name of the model to remove
    """
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        rospy.logwarn("Call the method for removing the model: " + model)
        remove_model_proxy = rospy.ServiceProxy(
            '/gazebo/delete_model', DeleteModel)
        remove_model_proxy(model)
    except rospy.ServiceException, ex:
        print "Service call delete_model failed: %e" % ex


def choose_random_ground():
    """
    Return the name of the new ground to select
    """

    ground_list = ["sand1", "sand2", "sand3",
                   "snow1", "snow2", "snow3",
                   "grass1", "grass2", "grass3",
                   "asphalt1", "asphalt2", "asphalt3",
                   "soil1", "soil2", "soil3"]

    ground_index = STDrandom.randint(0, len(ground_list) - 1)
    ground = ground_list[ground_index]
    return ground


def generate_new_world(model_to_add):
    """
    Remove the old model on the floor and add new ones.

    @param model_to_remove is the name of the model to remove (all its istances)
    @param model_to_add is the name of the model to add
    """
    #  First remove all the instance of the old model
    #  25 is the maximum number of istances of the model in the world

    ground_list = ["sand1", "sand2", "sand3",
                   "snow1", "snow2", "snow3",
                   "grass1", "grass2", "grass3",
                   "asphalt1", "asphalt2", "asphalt3",
                   "soil1", "soil2", "soil3"]

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


def update_quadrotor_pose(quadrotor_pose, done_reward):
    """
    Update the current pose of the quadrotor

    @param quadrotor_pose is the current pose
    @param x,y,z,orientation_x, orientation_y, orientation_z, orientation_w are cartesian coordinate of the new pose
    """
    quadrotor_pose.pose.position.x = done_reward.x
    quadrotor_pose.pose.position.y = done_reward.y
    quadrotor_pose.pose.position.z = done_reward.z
    quadrotor_pose.pose.orientation.x = done_reward.orientation_x
    quadrotor_pose.pose.orientation.y = done_reward.orientation_y
    quadrotor_pose.pose.orientation.z = done_reward.orientation_z
    quadrotor_pose.pose.orientation.w = done_reward.orientation_w
    return quadrotor_pose


def adjust_altitude(altitude):
    """
    Adjust the altitude of the quadrotor in order to make it descend one meter each time.

    @param altitude is the current UAV's altitude
    """
    new_altitude = round(altitude)
    new_altitude = new_altitude - 1.0
    return new_altitude

# if __name__ == "__main__":


def main():
    """
    Initialize and run the rospy node
    """
    timer_total_start = time.time()
    rospy.init_node("ReplayBufferFiller")
    rospy.loginfo("----- Replay Buffer Filler -----")

    replay_memory_size = 20000
    replay_buffer_path = "./replay_buffer_shared.pickle"
    replay_buffer_path_neutral = "./replay_buffer_neutral.pickle"
    replay_buffer_path_positive = "./replay_buffer_positive.pickle"
    replay_buffer_path_negative = "./replay_buffer_negative.pickle"
    replay_buffer = ExperienceReplayBuffer(capacity=replay_memory_size)
    replay_buffer_neutral = ExperienceReplayBuffer(capacity=replay_memory_size)
    replay_buffer_positive = ExperienceReplayBuffer(
        capacity=replay_memory_size)
    replay_buffer_negative = ExperienceReplayBuffer(
        capacity=replay_memory_size)
    # Load the Replay buffer from file or accumulate experiences
    if(os.path.isfile(replay_buffer_path) == True):
        print("Replay buffer loading from file: " +
              str(replay_buffer_path))
        replay_buffer.load(replay_buffer_path)
    else:
        print('No buffer_1 found')

    if(os.path.isfile(replay_buffer_path_neutral) == True):
        print("Replay buffer loading from file: " +
              str(replay_buffer_path_neutral))
        replay_buffer_neutral.load(replay_buffer_path_neutral)
    else:
        print('No buffer_2 found')

    if(os.path.isfile(replay_buffer_path_positive) == True):
        print("Replay buffer loading from file: " +
              str(replay_buffer_path_positive))
        replay_buffer_positive.load(replay_buffer_path_positive)
    else:
        print('No buffer_2 found')

    if(os.path.isfile(replay_buffer_path_negative) == True):
        print("Replay buffer loading from file: " +
              str(replay_buffer_path_negative))
        replay_buffer_negative.load(replay_buffer_path_negative)
    else:
        print('No buffer_2 found')

    # Create a subscriber fot the greyscale image
    rospy.Subscriber(
        "/quadrotor/ardrone/bottom/ardrone/bottom/image_raw", ROSImage, image_callback)

    images_stack_size = 4
    tot_steps = 3000000  # finite-horizont simulation
    frame_preliminary = 0

    saving_every_tot_experiences = 250
    is_buffer_saved = True
    noop_time = 2.0  # pause in seconds between actions
    steps_per_episodes = 30
    total_experience_counter = 0.0
    episode = 0
    start_episode = 1
    tot_episodes = 20000
    wrong_altitude = False
    quadrotor_pose = ModelState()
    quadrotor_pose.model_name = "quadrotor"
    quadrotor_pose.reference_frame = "world"
    for episode in range(start_episode, tot_episodes):
        cumulated_reward = 0
        print ""
        print "Preliminary Episode: " + str(episode)
        # print "Ground counter : " + str(ground_counter)
        # Reset UAV at random pose
        reset_pose()
        send_action('stop')
        rospy.sleep(3.0)
        # get_image()
        image_t = _last_image
        # When the replay buffer is empty, fill it with the same picture 4
        # times
        image_t = np.stack([image_t] * images_stack_size,
                           axis=2)  # create a stack of X images
        timer_start = time.time()
        actual_time = rospy.get_rostime()
        rospy_start_time = actual_time.secs + actual_time.nsecs / 1000000000.0
        frame_episode = 0

        done_reward = get_done_reward()
        update_quadrotor_pose(quadrotor_pose, done_reward)

        for step in range(tot_steps):
            # Execute a random action in the world and observe the reward and
            # state_t1.
            action = get_random_action()
            send_action(action)
            if action == "descend":
                rospy.sleep(5.0)
                send_action("stop")
                rospy.sleep(1.0)
            else:
                rospy.sleep(noop_time)
            # Acquire a new frame and convert it in a numpy array
            image_t1 = _last_image
            done_reward = get_done_reward()
            # NOTE: moved here to fix problem with baricenter (partially
            # reduced)
            send_action("stop")

            # Get the reward and done status

            reward = done_reward.reward
            done = done_reward.done
            print "Step(" + str(step) + "), Action: " + action + ", Altitude: " + str(done_reward.z) + ", Reward: " + str(reward)
            wrong_altitude = done_reward.wrong_altitude
            if wrong_altitude == True:
                rospy.logerr("[ERROR] Wrong altitude!")
            # Calculate the new cumulated_reward
            cumulated_reward += reward
            # state_t1, reward, done, info = env.step(action)
            image_t1 = np.expand_dims(image_t1, 2)
            # stack the images
            image_t1 = np.append(image_t[:, :, 1:], image_t1, axis=2)
            # Store the experience in the replay buffer
            if reward > 0:
                if action == "descend":
                    replay_buffer_positive.add_experience(
                        image_t, action, reward, image_t1, done)
                    is_buffer_saved = False
                    # pass
                else:
                    rospy.logerr(
                        "[POSITIVE]Wrong action for positive reward: %s", action)
            elif reward == -1.0:
                if action == "descend":
                    replay_buffer_negative.add_experience(
                        image_t, action, reward, image_t1, done)
                    # pass
                else:
                    rospy.logerr(
                        "[NEGATIVE]Wrong action for negative reward: %s", action)
            else:
                # pass
                replay_buffer_neutral.add_experience(
                    image_t, action, reward, image_t1, done)
            # Add all the experiences without distinction into the shared
            # buffer
            replay_buffer.add_experience(
                image_t, action, reward, image_t1, done)

            frame_preliminary += 1  # To call every time a frame is obtained
            total_experience_counter += 1
            image_t = image_t1
            timer_episode_stop = time.time()
            frame_episode += 1
            update_quadrotor_pose(quadrotor_pose, done_reward)

            # rospy.sleep(2.0) #NOTE: fix the descend bug affecting the
            # altitude
            if frame_episode >= steps_per_episodes:
                done = True
            # Save the buffer every 25000 experiences
            # if replay_buffer_positive.return_size() %
            # saving_every_tot_experiences == 0 and is_buffer_saved == False:
            if total_experience_counter % saving_every_tot_experiences == 0:
                timer_start = time.time()
                print("")
                print("Saving the replay buffer in: " + replay_buffer_path)
                print("Sit back and relax, it may take a while...")
                replay_buffer.save(replay_buffer_path)
                timer_stop = time.time()
                print "Time episode: " + str(timer_stop - timer_start) + " seconds"
                print "Time episode: " + str((timer_stop - timer_start) / 60) + " minutes"
                print("Done!")
                timer_start = time.time()
                print("")
                print("Saving the replay buffer in: " +
                      replay_buffer_path_neutral)
                print("Sit back and relax, it may take a while...")
                replay_buffer_neutral.save(replay_buffer_path_neutral)
                timer_stop = time.time()
                print "Time episode: " + str(timer_stop - timer_start) + " seconds"
                print "Time episode: " + str((timer_stop - timer_start) / 60) + " minutes"
                print("Done!")
                print("")
                print("Saving the replay buffer in: " +
                      replay_buffer_path_positive)
                print("Sit back and relax, it may take a while...")
                replay_buffer_positive.save(replay_buffer_path_positive)
                timer_stop = time.time()
                print "Time episode: " + str(timer_stop - timer_start) + " seconds"
                print "Time episode: " + str((timer_stop - timer_start) / 60) + " minutes"
                print("Done!")
                print("")
                print("Saving the replay buffer in: " +
                      replay_buffer_path_negative)
                print("Sit back and relax, it may take a while...")
                replay_buffer_negative.save(replay_buffer_path_negative)
                timer_stop = time.time()
                print "Time episode: " + str(timer_stop - timer_start) + " seconds"
                print "Time episode: " + str((timer_stop - timer_start) / 60) + " minutes"
                print("Done!")
                print("")
                is_buffer_saved = True
            if done:
                episode += 1
                timer_stop = time.time()
                actual_time = rospy.get_rostime()
                rospy_stop_time = actual_time.secs + actual_time.nsecs / 1000000000.0
                rospy_time_elapsed = rospy_stop_time - rospy_start_time
                print "Replay Buffer Size: " + str(replay_buffer.return_size()) + " out of " + str(replay_memory_size)
                print "Replay Buffer Neutral Size: " + str(replay_buffer_neutral.return_size()) + " out of " + str(replay_memory_size)
                print "Replay Buffer Positive Size: " + str(replay_buffer_positive.return_size()) + " out of " + str(replay_memory_size)
                print "Replay Buffer Negative Size: " + str(replay_buffer_negative.return_size()) + " out of " + str(replay_memory_size)
                print "Frame counter: " + str(frame_preliminary)
                print "Time episode: " + str(timer_stop - timer_start) + " seconds"
                print("Ros time episode: " +
                      str(rospy_time_elapsed) + " seconds")
                if cumulated_reward >= 0:
                    rospy.logwarn("Positive reward obtained!")
                print "Cumulated reward: " + str(cumulated_reward)
                print "Episode finished after {} timesteps".format(step + 1)
                break


if __name__ == "__main__":
    main()
