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

def generate_new_world(model_to_add, ground_list):
    """
    Remove the old model on the floor and add new ones.

    @param model_to_remove is the name of the model to remove (all its istances)
    @param model_to_add is the name of the model to add
    """
    
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
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try：
        os.system("rosrun gazebo_ros spawn_model -file ~/.gazebo/models/" + model_to_add +"/model.sdf -sdf -model " + model_to_add + "_plane -x 0 -y 0")
    except: 
        print "Impossible to import the ground"
        rospy.signal_shutdown("Shutdown!")

def choose_random_ground(ground_list):
    """
    Return the name of the new ground to select
    """
                   
    ground_index = STDrandom.randint(0, len(ground_list)-1)
    ground = ground_list[ground_index]
    return ground

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
    probability = (1.0 - probability_descend)/ (len(action_list) -1)
    action_probability = [probability, probability, probability, probability, probability, probability_descend]
    action = np.random.choice(action_list, 1, p=action_probability)[0]
    #action_index = STDrandom.randint(0, 10)
    #action = action_list[action_index]

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
        reset_pose_proxy = rospy.ServiceProxy('/drl/set_model_state', ResetPosition)
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
    policy_weights_path = ''
    target_weights_path = ''
    replay_buffer_path = ""
    replay_buffer_path_positive = ""
    replay_buffer_path_negative = ""

    
    ground_list = ["asphalt1", "asphalt2", "asphalt3",
                   "grass1", "grass2", "grass3",
                   "sand1", "sand2", "sand3",
                   "snow1", "snow2", "snow3"
                   "soil1", "soil2", "soil3"]

    #TODO: in case of crash: set episodes to last episode+1, frame to last frames, policy paths to last episode
    summary_folder = ""  # if empty the summary is written in ./log/ + current time
    start_episode = 1  # change to last episode number
    frame_counter = 0 # change to last number of frames
    is_double_dqn = True
    root_positive = ""
    root_neutral = ""
    root_negative = ""
    total_buffer_positive = 10
    total_buffer_neutral = 20
    total_buffer_negative = 10
    replay_buffer_path_positive = root_positive + "replay_buffer_positive_" + str(np.random.randint(1,total_buffer_positive+1)) +  ".pickle"
    replay_buffer_path_neutral = root_neutral + "replay_buffer_neutral_" + str(np.random.randint(1,total_buffer_neutral+1)) +  ".pickle"
    replay_buffer_path_negative = root_negative + "replay_buffer_negative_" + str(np.random.randint(1,total_buffer_negative+1)) +  ".pickle"
    if is_double_dqn:
        update_C = 30000 # target network update frequency (10.000 in original work)
        epsilon_stop = 0.01 #in the double DQN algorithm epsilon is decreased more
    else:
        update_C = 10000
        epsilon_stop = 0.1


    screen_width = 84  # original is 160
    screen_height = 84  # original is 210
    images_stack_size = 4
    # Use only the first 5 actions for this simulation
    # action_list = ['left', 'right', 'forward', 'backward', 'stop', 'descend']
    # 0 (left, 1 (right), 2 (forward), 3 (backward), 4 (stop), 5 (descend)
    tot_actions = 6
    batch_size = 32  # size of the experience batch
    batch_size_positive = 8
    batch_size_negative = 8
    tot_steps = 1000  # finite-horizont simulation
    # 10x10^6 high number since training can be stopped at every time
    tot_episodes = 10000000
    steps_per_episodes = 5  # expressed in number step
    noop_time = 2.0  # pause in seconds between actions
    render_step = 100  # render every xxx minutes
    # save the networks weights every X hourse [2000 episodes = 3 hours]
    save_network_step = 1500
    save_buffer_episode = 1500
    sync_buffer_episode = 1500
    save_replay_buffer = True
    # 1.000.000 in original work [60.000 takes 29GB of ram, 40.000 takes 19GB]
    replay_memory_size = 400000
    replay_positive_counter = 0
    replay_negative_counter = 0
    # Steps with random actions to fill the replay buffer.
    # In the original paper the limit is expressed in frames after 50.000 frame
    # the real learning starts (it roughly corresponds to 500 episodes)
    # expressed in experiences #TODO CHANGE BACK TO 50000
    epsilon = 1.0
    epsilon_start = 1.0
    # (in frame counted) after this point epsilon=0.1 (it is 1.000.000 in original work)
    epsilon_steps = 500000
    epsilon_array = np.linspace(
        start=epsilon_start, stop=epsilon_stop, num=epsilon_steps, endpoint=True)
    discount_factor = 0.99

    num_ground_plane = 70
    #frame_per_ground_plane = replay_memory_size / num_ground_plane
    actual_ground_index = 0
    # episode_per_ground specify the number of episodes with the same ground plane
    ground_counter = 1
    episode_per_ground = 2

    timer_total_start = time.time()
    rospy.init_node("DRLTrainingNode")
    rospy.loginfo("----- DRL Training Node -----")
    
    # Create a subscriber fot the greyscale image
    # rospy.Subscriber("/drl/grey_camera", ROSImage, image_callback)#TODO
    # restore default
    rospy.Subscriber(
        "/quadrotor/ardrone/bottom/ardrone/bottom/image_raw", ROSImage, image_callback,queue_size=30)  # Store the last 30 messages before discarding them

    images_stack_size = 4
    tot_steps = 3000000  # finite-horizont simulation

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
    replay_buffer_neutral = ExperienceReplayBuffer(capacity=50000)
    replay_buffer_positive = ExperienceReplayBuffer(capacity=50000)
    replay_buffer_negative = ExperienceReplayBuffer(capacity=50000)
    #tmp_buffer_positive = ExperienceReplayBuffer(capacity=100000)
    #tmp_buffer_negative = ExperienceReplayBuffer(capacity=100000)

    # Instructions for updating: Use `tf.global_variables_initializer
    init = tf.global_variables_initializer()
    sess.run(init)


    # Load Neural Networks weights from memory if a valid checkpoint path is passed
    if(policy_weights_path != "" and target_weights_path != ""):
        print("Loading weights from memory...")
        policy_network.load_weights(policy_weights_path)
        target_network.load_weights(target_weights_path)
        epsilon_start = 0.1
    else:
        print("The networks path are empty. Learning from scratch...")

    # Load the Replay buffer from file or accumulate experiences
    if(os.path.isfile(replay_buffer_path_neutral) == True): 
        print("Replay buffer loading from file: " +
              str(replay_buffer_path_neutral))
        replay_buffer_neutral.load(replay_buffer_path_neutral)
    else:
	    raise Exception('No buffer found')

    if(os.path.isfile(replay_buffer_path_positive) == True): 
        print("Replay buffer loading from file: " +
              str(replay_buffer_path_positive))
        replay_buffer_positive.load(replay_buffer_path_positive)
    else:
	    raise Exception('No buffer found') 

    if(os.path.isfile(replay_buffer_path_negative) == True): 
        print("Replay buffer loading from file: " +
              str(replay_buffer_path_negative))
        replay_buffer_negative.load(replay_buffer_path_negative)
    else:
	    raise Exception('No buffer found') 
        

    # start here
    for episode in range(start_episode, tot_episodes):
        # Reset the ground in the environment every 50 episodes (or episode_per_ground)
        if(ground_counter < episode_per_ground):
            ground_counter = ground_counter + 1
        else:
            ground = choose_random_ground(ground_list)
            generate_new_world(ground, ground_list)
            ground_counter = 1

            # Save the actual positive replay buffer and load a new one
            print "Saving positive buffer replay in " + str(replay_buffer_path_positive)
            replay_buffer_positive.save(replay_buffer_path_positive)
            replay_buffer_path_positive = root_positive + "replay_buffer_positive_" + str(np.random.randint(1,total_buffer_positive+1)) + ".pickle"
            print "Loading a new positive buffer replay from " + str(replay_buffer_path_positive)
            replay_buffer_positive.load(replay_buffer_path_positive)

            print "Saving neutral buffer replay in " + str(replay_buffer_path_neutral)
            replay_buffer_neutral.save(replay_buffer_path_neutral)
            replay_buffer_path_neutral = root_neutral + "replay_buffer_neutral_" + str(np.random.randint(1,total_buffer_neutral+1)) + ".pickle"
            print "Loading a new neutral buffer replay from " + str(replay_buffer_path_neutral)
            replay_buffer_neutral.load(replay_buffer_path_neutral)

            print "Saving negative buffer replay in " + str(replay_buffer_path_negative)
            replay_buffer_negative.save(replay_buffer_path_negative)
            replay_buffer_path_negative = root_negative + "replay_buffer_negative_" + str(np.random.randint(1,total_buffer_negative+1)) + ".pickle"
            print "Loading a new negative buffer replay from " + str(replay_buffer_path_negative)
            replay_buffer_negative.load(replay_buffer_path_negative)
            print("")

        timer_start = time.time()
        actual_time = rospy.get_rostime()
        rospy_start_time = actual_time.secs + actual_time.nsecs / 1000000000.0
        frame_episode = 0
        cumulated_reward = 0
        epsilon_used = 0
        print("")
        print("Episode: " + str(episode))
        print("Positive replay buffer used:" + str(replay_buffer_path_positive))
        print("Neutral replay buffer used:" + str(replay_buffer_path_neutral))
        print("Negative replay buffer used:" + str(replay_buffer_path_negative))
        # 1-Accumulate the first state
        reset_pose()
        print "Reset pose!"
        send_action("stop")
        rospy.sleep(3.0)
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
            epsilon_used_bool = False
            if(np.random.random_sample(1) < epsilon):
                epsilon_used += 1
                action = get_random_action()
                epsilon_used_bool = True
            # Take the action from the policy network
            else:
                # Here image_t is always ready and it can be given as input to
                # the network
                action_distribution = policy_network.return_action_distribution(
                    input_data=np.reshape(image_t, (1, 84, 84, images_stack_size)), softmax=False)
                action = np.argmax(action_distribution)
                action = convert_action_int_to_str(action)
                
            send_action(action)
            if action == "descend":
                rospy.sleep(5.0)
                send_action("stop")
                rospy.sleep(1.0)
            else:
                rospy.sleep(noop_time)
            
            # 3- Get the state_t1 repeating the action obtained at step-2 and add everything
            # in the replay buffer. Then set state_t = state_t1
            image_t1 = _last_image
            done_reward = get_done_reward()
            send_action("stop") #NOTE: fix for baricenter
            
            reward = done_reward.reward
            done = done_reward.done

            if epsilon_used_bool:
                print " Step(" + str(step) + "), Action: " + action + ", Altitude: " + str(done_reward.z) + ", Reward: " + str(reward)
            else:
                print "#Step(" + str(step) + "), Action: " + action + ", Altitude: " + str(done_reward.z) + ", Reward: " + str(reward)
                # ['left', 'right', 'forward', 'backward', 'stop', 'land'
                print(" left:" + str(action_distribution[0][0][0]) + "; right:" + str(action_distribution[0][0][1]) + "; forward:" + str(action_distribution[0][0][2]) + "; backward:" + str(action_distribution[0][0][3]) + "; stop:" + str(action_distribution[0][0][4])  + "; descend:" + str(action_distribution[0][0][5]))
            wrong_altitude = done_reward.wrong_altitude
            if wrong_altitude == True:
                rospy.logerr("[ERROR] Wrong altitude!")
                
            image_t1 = np.expand_dims(image_t1, 2)
            # stack the images
            image_t1 = np.append(image_t[:, :, 1:], image_t1, axis=2)
            if reward > 0:
                if action == "descend":
		            # replay_buffer_positive.add_experience(image_t, action, reward, image_t1, done)
                    replay_buffer_positive.add_experience_and_rotate(image_t, action, reward, image_t1, done, rotation_list=['90', '180', '270', 'vflip', 'vflip90', 'vflip180', 'vflip270'])
                else:
                    rospy.logerr("[POSITIVE]Wrong action for positive reward: %s", action)
            elif reward == -1.0:
                if action == "descend":
                    replay_buffer_negative.add_experience(image_t, action, reward, image_t1, done)
                else:
                    rospy.logerr("[NEGATIVE]Wrong action for negative reward: %s", action)
            else:
                replay_buffer_neutral.add_experience(image_t, action, reward, image_t1, done)
            frame_counter += 1  # To call every time a frame is obtained
            frame_episode += 1
            image_t = image_t1
            cumulated_reward += reward
            #send_action("stop") #TODO: original position

            # 4- Sampling a random mini-batch from the Replay Buffer
            experience_batch = replay_buffer_neutral.return_experience_batch(batch_size=batch_size - batch_size_positive - batch_size_negative)
            experience_batch_positive = replay_buffer_positive.return_experience_batch(batch_size=batch_size_positive)
            experience_batch_negative = replay_buffer_negative.return_experience_batch(batch_size = batch_size_negative)
            experience_batch = experience_batch + experience_batch_positive + experience_batch_negative

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

            #Here it is decided if using Double DQN or not
            if is_double_dqn:
                # Calling the policy network on image_t1
                q_values_policy_t1 = policy_network.return_action_distribution(image_t1_batch, softmax=False)
                q_values_policy_t1 = np.asarray(q_values_policy_t1)
                q_values_policy_t1 = np.reshape(q_values_policy_t1, (batch_size, tot_actions)) # shape (batch_size, tot_actions)
                # Getting the argmax
                indices_argmax_policy = np.argmax(q_values_policy_t1, axis=1) # shape (batch_size)
                # Now calling the target on image_t1
                q_values_t1 = target_network.return_action_distribution(image_t1_batch, softmax=False)  # shape (batch_size, tot_actions)
                q_values_t1 = np.asarray(q_values_t1)
                q_values_t1 = np.reshape(q_values_t1, (batch_size, tot_actions))
                # Using Numpy indexing on q_values_t1 to return the q_values we need using policy_argmax as indeces
                target_batch = reward_t_batch + np.invert(done_t1_batch).astype(np.float32) * discount_factor * q_values_t1[np.arange(q_values_t1.shape[0]),indices_argmax_policy]
                target_batch = np.reshape(target_batch, (batch_size))
                if DEBUG: print "Double DQN is running..."
            else:
                # The Q-values at t+1 returned by the Target_Network are used to find the discounted rewards
                q_values_t1 = target_network.return_action_distribution(image_t1_batch, softmax=False)  # shape (batch_size, tot_actions)
                q_values_t1 = np.asarray(q_values_t1)
                q_values_t1 = np.reshape(q_values_t1, (batch_size, tot_actions))
                # Finding the discounted rewards
                target_batch = reward_t_batch + np.invert(done_t1_batch).astype(np.float32) * discount_factor * np.amax(q_values_t1, axis=1)
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
                print("Ros time episode: " + str(rospy_time_elapsed) + " seconds")
                print("Replay Buffer experiences: " + str(replay_buffer_neutral.return_size()))
                print("Replay Buffer Positive experiences: " + str(replay_buffer_positive.return_size()))
                print("Replay Buffer Negative experiences: " + str(replay_buffer_negative.return_size()))
                print("Double DQN used: " + str(is_double_dqn))
                print("Epsilon: " + str(epsilon))
                print("Epsilon used: " + str(epsilon_used) + " out of " + str(step + 1) + "(" + str(float((epsilon_used * 100.0) / (step + 1.0))) + "%)")
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
            target_network.save_weights( target_folder + "target_checkpoint.ckp")
            # Checking the policy folder and saving
            policy_folder = "./checkpoint/episode_" + str(episode) + "/policy/"
            if not os.path.exists(policy_folder):
                os.makedirs(policy_folder)
            policy_network.save_weights( policy_folder + "policy_checkpoint.ckp")
            # Save the Replay Buffer
            #timer_start = time.time()
            #print("")
            #print("Saving the replay buffer in: " + replay_buffer_path)
            #print("Sit back and relax, it may take a while...")
            #replay_buffer_neutral.save(replay_buffer_path)
            #timer_stop = time.time()
            #print "Time episode: " + str(timer_stop - timer_start) + " seconds"
            #print "Time episode: " + str((timer_stop - timer_start) / 60) + " minutes"
            #print("Done!")
            #timer_start = time.time()
            #print("")
            #print("Saving the positive replay buffer in: " + replay_buffer_path_positive)
            #print("Sit back and relax, it may take a while...")
            #replay_buffer_positive.save(replay_buffer_path_positive)
            #timer_stop = time.time()
            #print "Time episode: " + str(timer_stop - timer_start) + " seconds"
            #print "Time episode: " + str((timer_stop - timer_start) / 60) + " minutes"
            #print("Done!")
            #print("")
            #print("Saving the negative replay buffer in: " + replay_buffer_path_negative)
            #print("Sit back and relax, it may take a while...")
            #replay_buffer_negative.save(replay_buffer_path_negative)
            #timer_stop = time.time()
            #print "Time episode: " + str(timer_stop - timer_start) + " seconds"
            #print "Time episode: " + str((timer_stop - timer_start) / 60) + " minutes"
            #print("Done!")
            #print("")
            sys.stdout.flush()


if __name__ == "__main__":
    main()
