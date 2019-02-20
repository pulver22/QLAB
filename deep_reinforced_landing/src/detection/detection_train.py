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
from deep_reinforced_landing.srv import NewCameraService, GetDoneAndReward, SendCommand, ResetPosition # DRL services
from gazebo_msgs.srv import DeleteModel # Gazebo service for removing a model
import rospy
# Rename to avoid confusion with Image lib
from sensor_msgs.msg import Image as ROSImage
import subprocess # needed for using bash command


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
        remove_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
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
    ground_list = ["asphalt1","asphalt2","asphalt3","asphalt4","asphalt5","asphalt6","asphalt7","asphalt8","asphalt9","asphalt10",
                   "brick1","brick2","brick3","brick4","brick5","brick6","brick7","brick8","brick9","brick10",
                   "grass1","grass2","grass3","grass4","grass5","grass6","grass7","grass8","grass9","grass10",
                   "pavement1","pavement2","pavement3","pavement4","pavement5","pavement6","pavement7","pavement8","pavement9","pavement10",
                   "sand1","sand2","sand3","sand4","sand5","sand6","sand7","sand8","sand9","sand10",
                   "snow1","snow2","snow3","snow4","snow5","snow6","snow7","snow8","snow9","snow10",
                   "soil1","soil2","soil3","soil4","soil5","soil6","soil7","soil8","soil9","soil10"]

    for ground in ground_list:
        try:
            ground = ground + "_plane"
            remove_model(ground)
        except:
            pass
    
    if type(model_to_add) is int:
        #print "Is an int!"
        model_to_add = ground_list[model_to_add] 
    
    print "\n"
    rospy.logwarn( "Ground choosen is " + str(model_to_add) )   
    # Spawn new istances for the new model
    os.system("rosrun gazebo_ros spawn_model -file $GAZEBO_MODEL_PATH/" + model_to_add +"/model.sdf -sdf -model " + model_to_add + "_plane -x 0 -y 0")

def choose_random_ground():
    """
    Return the name of the new ground to select
    """

    ground_list = ["asphalt1","asphalt2","asphalt3","asphalt4","asphalt5","asphalt6","asphalt7","asphalt8","asphalt9","asphalt10",
                   "brick1","brick2","brick3","brick4","brick5","brick6","brick7","brick8","brick9","brick10",
                   "grass1","grass2","grass3","grass4","grass5","grass6","grass7","grass8","grass9","grass10",
                   "pavement1","pavement2","pavement3","pavement4","pavement5","pavement6","pavement7","pavement8","pavement9","pavement10",
                   "sand1","sand2","sand3","sand4","sand5","sand6","sand7","sand8","sand9","sand10",
                   "snow1","snow2","snow3","snow4","snow5","snow6","snow7","snow8","snow9","snow10",
                   "soil1","soil2","soil3","soil4","soil5","soil6","soil7","soil8","soil9","soil10"]
                   
    ground_index = STDrandom.randint(0, len(ground_list)-1)
    ground = ground_list[ground_index]
    return ground

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
        get_random_action_proxy = rospy.ServiceProxy('/drl/send_command', SendCommand)
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
    # "/home/thebeast/DRL/src/test_gym_breakout/checkpoint/_episode_1000/policy/policy_checkpoint.ckp"
    policy_weights_path = '/home/pulver/Desktop/simulation_3b/checkpoint/episode_86400/policy/policy_checkpoint.ckp'
    # "/home/thebeast/DRL/src/test_gym_breakout/checkpoint/_episode_1000/target/target_checkpoint.ckp"
    target_weights_path = '/home/pulver/Desktop/simulation_3b/checkpoint/episode_86400/target/target_checkpoint.ckp'
    replay_buffer_path = "./replay_buffer.pickle"
    #eplay_buffer_path = "./replay_buffer.pickle"
    # "./preliminary_replay_buffer.pickle" #TODO remote test
    preliminary_replay_buffer_path = "./replay_buffer.pickle"
    #preliminary_replay_buffer_path = "./replay_buffer.pickle"
    summary_folder = ""  # if empty the summary is written in ./log/ + current time
    start_episode = 86401  # change to last episode number
    frame_counter = 962945  # change to last number of frames

    frame_preliminary = 0
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
    tot_episodes = 10000000
    steps_per_episodes = 20  # expressed in number step
    noop_time = 2.0  # pause in seconds between actions
    render_step = 100  # render every xxx minutes
    # save the networks weights every X hourse [2000 episodes = 3 hours]
    save_network_step = 1800
    save_replay_buffer = True
    # target network update frequency (10.000 in original work)
    update_C = 10000
    # 1.000.000 in original work [60.000 takes 29GB of ram, 40.000 takes 19GB]
    replay_memory_size = 400000
    # Steps with random actions to fill the replay buffer.
    # In the original paper the limit is expressed in frames after 50.000 frame
    # the real learning starts (it roughly corresponds to 500 episodes)
    # expressed in experiences #TODO CHANGE BACK TO 50000
    replay_memory_preliminary_size = 100
    epsilon = 1.0
    epsilon_start = 1.0
    epsilon_stop = 0.1
    # (in frame counted) after this point epsilon=0.1 (it is 1.000.000 in original work)
    epsilon_steps = 500000
    epsilon_array = np.linspace(
        start=epsilon_start, stop=epsilon_stop, num=epsilon_steps, endpoint=True)
    discount_factor = 0.99

    num_ground_plane = 70
    frame_per_ground_plane = replay_memory_size / num_ground_plane
    actual_ground_index = 0
    # episode_per_ground specify the number of episodes with the same ground plane
    ground_counter = 1
    episode_per_ground = 50

    timer_total_start = time.time()
    rospy.init_node("DeepReinforcedLanding")
    rospy.loginfo("----- Deep Reinforced Landing Node -----")
    replay_memory_size = 400000
    replay_buffer_path = "./replay_buffer.pickle"
    #replay_buffer_path = "./replay_buffer.pickle"
    replay_buffer = ExperienceReplayBuffer(capacity=replay_memory_size)

    # Create a subscriber fot the greyscale image
    # rospy.Subscriber("/drl/grey_camera", ROSImage, image_callback)#TODO
    # restore default
    rospy.Subscriber(
        "/quadrotor/ardrone/bottom/ardrone/bottom/image_raw", ROSImage, image_callback,queue_size=30)  # Store the last 30 messages before discarding them

    images_stack_size = 4
    tot_steps = 3000000  # finite-horizont simulation
    frame_preliminary = 0

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
    target_network = QNetwork(sess, tot_actions=tot_actions, image_shape=(
        screen_width, screen_height, images_stack_size), batch_size=batch_size, network_name="target_net")
    replay_buffer = ExperienceReplayBuffer(capacity=replay_memory_size)
    # WARNING:tensorflow:From main.py:70: initialize_all_variables (from tensorflow.python.ops.variables) is deprecated and will be removed after 2017-03-02.
    # Instructions for updating: Use `tf.global_variables_initializer
    # init = tf.initialize_all_variables() #deprecated
    init = tf.global_variables_initializer()
    sess.run(init)

    # Synchronise the weights of the two networks (Optional)
    # target_network.set_weights(copy_from_network=policy_network)

    # Load Neural Networks weights from memory if a valid checkpoint path is passed
    # if(os.path.isfile(policy_weights_path) == True and
    # os.path.isfile(target_weights_path) == True):
    if(policy_weights_path != "" and target_weights_path != ""):
        print("Loading weights from memory...")
        policy_network.load_weights(policy_weights_path)
        target_network.load_weights(target_weights_path)
        epsilon_start = 0.1
    else:
        print("The networks path are empty. Learning from scratch...")

    # Load the Replay buffer from file or accumulate experiences
    if(os.path.isfile(preliminary_replay_buffer_path) == True):  # TODO set true
        print("Replay buffer loading from file: " +
              str(preliminary_replay_buffer_path))
        replay_buffer.load(preliminary_replay_buffer_path)
        print("Replay buffer memory uses (KB): " +
              str(replay_buffer.return_size_byte(value="kilobyte")))
        print("Replay buffer memory uses (MB): " +
              str(replay_buffer.return_size_byte(value="megabyte")))
        print("Replay buffer memory uses (GB): " +
              str(replay_buffer.return_size_byte(value="gigabyte")))
    else:
        print("No Replay buffer file found, accumulating experiences...")
        # It is necessary to fill the replay buffer with some experiences.
        # Here random actions are taken until 'replay_memory_preliminary_size'
        # is reached.
        
        # Reset the ground in the environment every 25k frames
        # ground_index = replay_buffer.return_size() / frame_per_ground_plane
        # if ground_index != actual_ground_index:
        #     generate_new_world(ground_index)
        # actual_ground_index = ground_index

        # Reset the ground in the environment every 50 episodes (or episode_per_ground)
        if(ground_counter < episode_per_ground):
            ground_counter = ground_counter + 1
        else:
            ground = choose_random_ground()
            generate_new_world(ground)
            ground_counter = 1 

        episode = 1
        while True:
            if replay_buffer.return_size() >= replay_memory_preliminary_size:
                break
            cumulated_reward = 0
            print ""
            print "Preliminary Episode: " + str(episode)
            # Reset UAV at random pose
            reset_pose()
            send_action('stop')
            rospy.sleep(1.0)
#            get_image()
            image_t = _last_image
            # When the replay buffer is empty, fill it with the same
            # picture 4 times
            # create a stack of X images
            image_t = np.stack([image_t] * images_stack_size, axis=2)
            timer_start = time.time()
            timer_episode_start = time.time()
            actual_time = rospy.get_rostime()
            rospy_start_time = actual_time.secs + actual_time.nsecs / 1000000000.0
            frame_episode = 0
            for step in range(tot_steps):
                # Execute a random action in the world and observe the
                # reward and state_t1.
                action = get_random_action()
                send_action(action)
                rospy.sleep(noop_time)
                # Acquire a new frame and convert it in a numpy array
#                get_image()
                image_t1 = _last_image
                #cv2.imshow("view", image_t1)
                # cv2.waitKey(1)
                # Get the reward and done status
                # If the action taken is to land and the UAV is inside the landing BB, done will be calculated accordingly
                done_reward = get_done_reward()
                reward = done_reward.reward
                done = done_reward.done
                # Calculate the new cumulated_reward
                cumulated_reward += reward
                # state_t1, reward, done, info = env.step(action)
                image_t1 = np.expand_dims(image_t1, 2)
                # stack the images
                image_t1 = np.append(image_t[:, :, 1:], image_t1, axis=2)
                # Sleep for a while and then acquire a second frame
                replay_buffer.add_experience( image_t, action, reward, image_t1, done)
                frame_preliminary += 1  # To call every time a frame is obtained
                #print frame_preliminary
                actual_time = rospy.get_rostime()
                rospy_stop_time = actual_time.secs + actual_time.nsecs / 1000000000.0
                rospy_time_elapsed = rospy_stop_time - rospy_start_time
                image_t = image_t1
                timer_episode_stop = time.time()
                frame_episode += 1
                send_action("stop")
                #if done == True:
                #    rospy.logwarn("Done True")
                #print "Action: " + str(action)
                #print "Reward: " + str(reward)
                #print "Done: " + str(done)
                #print "Frame number: " + str(frame_preliminary)
                #print "Ros episode time: " + str(rospy_time_elapsed)
                #print "Clock episode time: " + str(timer_episode_stop - timer_episode_start)
                #print ""
                if frame_episode >= steps_per_episodes:
                    done = True
                if done:
                    timer_stop = time.time()
                    episode += 1
                    print "Replay Buffer Size: " + str(replay_buffer.return_size()) + " out of " + str(replay_memory_size)
                    print "Replay buffer memory uses (KB): " + str(replay_buffer.return_size_byte(value="kilobyte"))
                    print "Replay buffer memory uses (MB): " + str(replay_buffer.return_size_byte(value="megabyte"))
                    print "Replay buffer memory uses (GB): " + str(replay_buffer.return_size_byte(value="gigabyte"))
                    print "Frame counter: " + str(frame_preliminary)
                    print "Time episode: " + str(timer_stop - timer_start) + " seconds"
                    # print "Total Time episode: " + str(timer_episode_stop
                    # - timer_episode_start) + " seconds"
                    print "Rospy time single step: " + str(rospy_time_elapsed / steps_per_episodes)
                    print "Replay Buffer experiences: " + str(replay_buffer.return_size())
                    print "Cumulated reward: " + str(cumulated_reward)
                    print "Episode finished after {} timesteps".format(step + 1)
                    break

        timer_start = time.time()
        if(save_replay_buffer == True):
            print("")
            print("Saving the replay buffer in: " + replay_buffer_path)
            print("Sit back and relax, it may take a while...")
            replay_buffer.save(replay_buffer_path)
            timer_stop = time.time()
            print "Time episode: " + str(timer_stop - timer_start) + " seconds"
            print "Time episode: " + str((timer_stop - timer_start) / 60) + " minutes"
            print("Done!")
            print("")

    # start here
    for episode in range(start_episode, tot_episodes):
        # Reset the ground in the environment every 25k frames
        # ground_index = replay_buffer.return_size() / frame_per_ground_plane
        # if ground_index != actual_ground_index:
        #     generate_new_world(ground_index)
        # actual_ground_index = ground_index

        # Reset the ground in the environment every 50 episodes (or episode_per_ground)
        if(ground_counter < episode_per_ground):
            ground_counter = ground_counter + 1
        else:
            ground = choose_random_ground()
            generate_new_world(ground)
            ground_counter = 1

        timer_start = time.time()
        actual_time = rospy.get_rostime()
        rospy_start_time = actual_time.secs + actual_time.nsecs / 1000000000.0
        frame_episode = 0
        cumulated_reward = 0
        epsilon_used = 0
        print("")
        print("Episode: " + str(episode))
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
            # With probability epsilon take random action otherwise it takes
            # an action from the policy network.
            if(frame_counter < epsilon_array.shape[0]):
                # takes epsilon from a linspace array
                epsilon = epsilon_array[frame_counter]
            else:
                epsilon = epsilon_stop
            # Take a random action
            if(np.random.random_sample(1) < epsilon):
                epsilon_used += 1
                action = get_random_action()
                send_action(action)
            # Take the action from the policy network
            else:
                # Here image_t is always ready and it can be given as input to
                # the network
                action_distribution = policy_network.return_action_distribution(
                    input_data=np.reshape(image_t, (1, 84, 84, images_stack_size)), softmax=False)
                action = np.argmax(action_distribution)
                action = convert_action_int_to_str(action)
                send_action(action)
            rospy.sleep(noop_time)
            # 3- Get the state_t1 repeating the action obtained at step-2 and add everything
            # in the replay buffer. Then set state_t = state_t1
            #get_image()
            image_t1 = _last_image
            # If the action taken is to land and the UAV is inside the landing BB, done will be calculated accordingly
            done_reward = get_done_reward()
            reward = done_reward.reward
            done = done_reward.done
            image_t1 = np.expand_dims(image_t1, 2)
            # stack the images
            image_t1 = np.append(image_t[:, :, 1:], image_t1, axis=2)
            replay_buffer.add_experience(image_t, action, reward, image_t1, done)
            frame_counter += 1  # To call every time a frame is obtained
            frame_episode += 1
            image_t = image_t1
            cumulated_reward += reward
            send_action("stop")

            # 4- Sampling a random mini-batch from the Replay Buffer
            experience_batch = replay_buffer.return_experience_batch(
                batch_size=batch_size)

            # 5- Evaluating the Target vector for the batch
            # image_t_batch = [x[0] for x in experience_batch] #experience_batch[0]
            # action_t_batch = np.reshape(np.asarray([x[1] for x in experience_batch]), (batch_size,1)) #experience_batch[1]
            # reward_t_batch = [x[2] for x in experience_batch] #experience_batch[2]
            # image_t1_batch = [x[3] for x in experience_batch] #experience_batch[3]
            # done_t1_batch = [x[4] for x in experience_batch]
            # #experience_batch[4]
            image_t_batch, action_t_batch, reward_t_batch, image_t1_batch, done_t1_batch = map(np.array, zip(*experience_batch))
            # Convert the action from str to int (needed by gradient descent
            # code)
            action_t_batch = convert_action_batch_str_to_int(action_t_batch)
            action_t_batch = np.reshape(action_t_batch, (batch_size))

            # The Q-values at t+1 returned by the Target_Network are used to
            # find the discounted rewards
            q_values_t1 = target_network.return_action_distribution(image_t1_batch, softmax=False)  # shape (batch_size, tot_actions)
            q_values_t1 = np.asarray(q_values_t1)
            q_values_t1 = np.reshape(q_values_t1, (batch_size, tot_actions))
            # Finding the discounted rewards
            # target_batch = reward_t_batch +
            # np.invert(done_t1_batch).astype(np.float32) *
            # np.reshape(discount_factor * np.amax(q_values_t1, axis=1),
            # (batch_size,1)) #shape (batch_size)
            target_batch = reward_t_batch + np.invert(done_t1_batch).astype(np.float32) * discount_factor * np.amax(q_values_t1, axis=1)
            #target_batch = np.expand_dims(target_batch, axis=1)
            target_batch = np.reshape(target_batch, (batch_size))

            if(DEBUG == True):
                print("Target Shape:")
                print(target_batch.shape)
                print("Action Shape:")
                print(action_t_batch.shape)
                print("Image Shape:")
                print(image_t_batch.shape)

            # 6- Perform a gradient descent step on policy network
            # Taking the action_t in state_t leads to state_t1 and reward_t
            # Passing: [image_t, actions_t, target_t], shapes: (batch_size,
            # 84,84, 4), (batch_size), (batch_size)
            summaries, filter_summaries = policy_network.perform_gradient_descent_step(
                input_data_batch=image_t_batch, action_data_batch=action_t_batch, target_data_batch=target_batch)
            tf_summary_writer.add_summary(summaries, frame_counter)

            # 7- Check if the C is reached and if true it updates the Target
            # Network
            if(frame_counter % update_C == 0):
                print("Updating the target network...")
                target_network.set_weights(copy_from_network=policy_network)

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
                local_summary = tf.Summary()
                local_summary.value.add(
                    simple_value=cumulated_reward, node_name="cumulated reward episode", tag="episode_reward")
                local_summary.value.add(
                    simple_value=step, node_name="steps episode", tag="episode_steps")
                local_summary.value.add(
                    simple_value=epsilon, node_name="epsilon", tag="epsilon")
                tf_summary_writer.add_summary(local_summary, frame_counter)
                tf_summary_writer.add_summary(
                    filter_summaries)  # the weights summary
                tf_summary_writer.flush()
                timer_stop = time.time()
                actual_time = rospy.get_rostime()
                rospy_stop_time = actual_time.secs + actual_time.nsecs / 1000000000.0
                rospy_time_elapsed = rospy_stop_time - rospy_start_time
                print("Tot Frame counter: " + str(frame_counter))
                print("Time episode: " + str(timer_stop - timer_start) + " seconds")
                print("Ros time episode: " +
                      str(rospy_time_elapsed) + " seconds")
                print("Replay Buffer experiences: " +
                      str(replay_buffer.return_size()))
                print("Epsilon: " + str(epsilon))
                print("Epsilon used: " + str(epsilon_used) + " out of " + str(step +
                                                                              1) + "(" + str(float((epsilon_used * 100.0) / (step + 1.0))) + "%)")
                if cumulated_reward >= 0:
                    rospy.logwarn("Positive reward obtained!")
                print("Cumulated reward: " + str(cumulated_reward))
                print("Episode finished after {} timesteps".format(step + 1))
                sys.stdout.flush()
                break

        # Save the networks weights every X episodes
        if(episode % save_network_step == 0):
            print("Saving the networks weights...")
            folder = "./checkpoint/episode_" + str(episode) + "/"
            if not os.path.exists(folder):
                os.makedirs(folder)
            # Checking the target folder and saving
            target_folder = "./checkpoint/episode_" + str(episode) + "/target/"
            if not os.path.exists(target_folder):
                os.makedirs(target_folder)
            target_network.save_weights(
                target_folder + "target_checkpoint.ckp")
            # Checking the policy folder and saving
            policy_folder = "./checkpoint/episode_" + str(episode) + "/policy/"
            if not os.path.exists(policy_folder):
                os.makedirs(policy_folder)
            policy_network.save_weights(
                policy_folder + "policy_checkpoint.ckp")
            # Save the preliminary Replay Buffer
            timer_start = time.time()
            if(save_replay_buffer == True):
                print("")
                print("Saving the replay buffer in: " + replay_buffer_path)
                print("Sit back and relax, it may take a while...")
                replay_buffer.save(replay_buffer_path)
                timer_stop = time.time()
                print "Time episode: " + str(timer_stop - timer_start) + " seconds"
                print "Time episode: " + str((timer_stop - timer_start) / 60) + " minutes"
                print("Done!")
                print("")
            sys.stdout.flush()


if __name__ == "__main__":
    main()
