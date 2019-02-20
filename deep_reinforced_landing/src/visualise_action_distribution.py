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
from deep_reinforced_landing.srv import NewCameraService, GetDoneAndReward, SendCommand, ResetPosition  # DRL services
from gazebo_msgs.srv import DeleteModel  # Gazebo service for removing a model
import rospy
# Rename to avoid confusion with Image lib
from sensor_msgs.msg import Image as ROSImage
import subprocess  # needed for using bash command
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import gridspec
import urllib
import thread
import threading
import subprocess as sp


DEBUG = False  # Set to False to disable the image shown at the begining
_last_image = None  # Global variable representing the last frame acquired by the camera
FFMPEG_BIN = "ffmpeg"  # on Linux ans Mac OS
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


def convert_action_int_to_str_many(action):
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


def acquire_frame(url):
    """
    Connect to a video stream and save a greyscale 84x84 frame.
    """
    # _video_stream = urllib.urlopen(url)
    # bytes = ''
    # while True:
    #     bytes += _video_stream.read(1024)
    #     a = bytes.find('\xff\xd8')
    #     b = bytes.find('\xff\xd9')
    #     if a != -1 and b != -1:
    #         # time.sleep(1)
    #         jpg = bytes[a:b + 2]
    #         bytes = bytes[b + 2:]
    #         image_t1 = cv2.imdecode(np.fromstring(
    #             jpg, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    #         # print image_t1.shape
    #         # Calculate the ratio when reducing row to 84
    #         r = 84.0 / image_t1.shape[0]
    #         dim = (int(image_t1.shape[1] * r), 84)
    #         # perform the actual resizing of the image and show it
    #         image_t1 = cv2.resize(image_t1, dim, interpolation=cv2.INTER_AREA)
    #         # print image_t1.shape
    #         # crop the image using array slices -- it's a NumPy array
    #         # after all!
    #         image_t1 = image_t1[:, 9:93]
    #         global _last_image
    #         _last_image = image_t1
    #         break

    # Camera values
    # intrinsic_matrix = np.array([[857.48296979, 0., 968.06224829], [
    #                             0., 876.71824265, 556.37145899], [0., 0., 1.0]])
    # distCoeff = b = np.array(
    #     [[-2.57614020e-01, 8.77086999e-02, -2.56970803e-04, -5.93390389e-04, -1.5194091e-02]])
    # crop = 0.0  # NOTE: a value of 0 discard black pixels, while a value of 1 keep all of them
    # width = 432
    # height = 240
    # size = (int(width), int(height))
    pipe = sp.Popen([FFMPEG_BIN, "-i", url,
                     "-loglevel", "quiet",  # no text output
                     "-an",   # disable audio
                     #"-vf", "lenscorrection=cx=0.5:cy=0.5:k1=-0.227:k2=-0.022",
                     "-f", "image2pipe",
                     "-pix_fmt", "bgr24",
                     "-vcodec", "rawvideo", "-"],
                    stdin=sp.PIPE, stdout=sp.PIPE)
    while True:
        # read 432*240*3 bytes (= 1 frame)
        print "------------"
        raw_image = pipe.stdout.read(432 * 240 * 3)
        image_t1 = np.fromstring(
            raw_image, dtype=np.uint8).reshape((240, 432, 3))
        #cv2.imwrite("Acquired.jpg", image_t1)
        cv2.imwrite("distorted_modified.jpg", image_t1)
        # Remove fisheye effect with bash and imagetricks
        os.system('mogrify -distort barrel "0 0 -0.14" distorted_modified.jpg')
        # # Load undistorted image
        gray_image = cv2.imread('distorted_modified.jpg', 0)
        # print "Image acquired: " + str(np.shape(image_t1))

        # gray_image = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)
        gray_image = gray_image[:, 96:336]  # Crop to image (240,240)
        image_t1 = cv2.resize(gray_image, (84, 84),
                              interpolation=cv2.INTER_AREA)
        cv2.imwrite("Final.jpg", image_t1)

        # Save the image distorted
        # cv2.imwrite("distorted.jpg", gray_image)
        # cv2.imwrite("distorted_modified.jpg", gray_image)
        # # Remove fisheye effect with bash and imagetricks
        # os.system('mogrify -distort barrel "0 0 -0.3" distorted_modified.jpg')
        # # Load undistorted image
        # gray_image = cv2.imread('distorted.jpg', 0)

        # print "Image after conversion to grayscale: " + str(np.shape(gray_image))
        # cv2.imwrite("Grayscale.jpg", gray_image)

        # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
        # intrinsic_matrix, distCoeff, size, alpha=crop,
        # centerPrincipalPoint=1)

        # mapx, mapy = cv2.initUndistortRectifyMap(
        #     intrinsic_matrix, distCoeff, None, newcameramtx, size, m1type=cv2.CV_32FC1)
        # gray_image = cv2.remap(gray_image, mapx, mapy, cv2.INTER_LINEAR)

        # undistort
        # gray_image = cv2.undistort(
        #     gray_image, intrinsic_matrix, distCoeff)
        # print "Image after undistort: " + str(np.shape(gray_image))
        # cv2.imwrite("Undistorted.jpg", gray_image)
        # crop the image
        # x, y, w, h = roi
        # gray_image = gray_image[y:y + 84, x:x + 84]
        # print "Image after crop: " + str(np.shape(gray_image))
        # cv2.imwrite("Cropped.jpg", gray_image)

        # Calculate the ratio when reducing row to 84
        # r = 84.0 / gray_image.shape[0]
        # dim = (int(gray_image.shape[1] * r), 84)
        # # perform the actual resizing of the image and show it
        # image_t1 = cv2.resize(gray_image, dim, interpolation=cv2.INTER_AREA)
        # # print np.shape(image_t1)
        # # crop the image using array slices -- it's a NumPy array
        # # after all!
        # image_t1 = image_t1[:, 33:117]
        # print np.shape(image_t1)

        # cv2.imshow("GoPro", image_t1)
        # if cv2.waitKey(5) == 27:
        #     break

        # NOTE: the following code should be used only if the camera is rotated
        # rotate ccw
        # image_t1 = cv2.transpose(image_t1)
        # image_t1 = cv2.flip(image_t1, flipCode=0)

        # rotate cw
        # image_t1 = cv2.transpose(image_t1)
        # image_t1 = cv2.flip(image_t1, flipCode=1)

        global _last_image
        _last_image = image_t1
        break

    return image_t1


def main():
    """
    Main function for training the DQN network in learning how to accomplish autonomous landing.
    """
    # ATTENTION: If you want to restore files from a previous simulation you
    # must pass valid values for these variables:
    policy_weights_path_1 = '/home/pulver/Desktop/episode_113250/policy/policy_checkpoint.ckp'
    root_images = "/home/pulver/Desktop/network_testing/" + \
        str(datetime.datetime.now().time()) + "/"
    # NOTE: openCV doesn't write in a folder that does not exist
    if not os.path.exists(root_images):
        os.makedirs(root_images)
    source = 3  # NOTE: 1 for real drone, 2 for gazebo, 3 for URL

    screen_width = 84  # original is 160
    screen_height = 84  # original is 210
    images_stack_size = 4
    # Use only the first 5 actions for this simulation
    # action_list = ['left', 'right', 'forward', 'backward', 'stop', 'descend']
    # 0 (left, 1 (right), 2 (forward), 3 (backward), 4 (stop), 5 (descend)
    tot_actions = 6
    batch_size = 32  # size of the experience batch

    rospy.init_node("DRLTrainingNode")
    rospy.loginfo("----- DRL Training Node -----")

    # Create a subscriber fot the greyscale image

    # 1) Real drone
    if source == 1:
        rospy.Subscriber("/drl/grey_camera", ROSImage, image_callback)
    elif source == 2:
        # 2) Gazebo
        rospy.Subscriber("/quadrotor/ardrone/bottom/ardrone/bottom/image_raw",
                         ROSImage, image_callback, queue_size=30)  # Store the last 30 messages
    elif source == 3:
        # 3) URL
        # video_stream_url = 'http://10.188.34.59:8080/videofeed'
        video_stream_url = "http://10.5.5.9:8080/live/amba.m3u8"
        bytes = ''
    else:
        print "Insert a correct source value (1 for real drone, 2 for gazebo, 3 for URL)"

    images_stack_size = 4
    tot_steps = 3000000  # finite-horizont simulation

    r = rospy.Rate(1)  # 10hz

    # Init session and networks
    sess = tf.Session()
    summary_folder = ""  # if empty the summary is written in ./log/ + current time
    if(summary_folder == ""):
        tf_summary_writer = tf.summary.FileWriter(
            './log/' + str(datetime.datetime.now().time()), sess.graph)
    else:
        tf_summary_writer = tf.summary.FileWriter(
            summary_folder, sess.graph)
    policy_network_1 = QNetwork(sess, tot_actions=tot_actions, image_shape=(
        screen_width, screen_height, images_stack_size), batch_size=batch_size,
        network_name="policy_net")

    # Instructions for updating: Use `tf.global_variables_initializer
    init = tf.global_variables_initializer()
    sess.run(init)

    # Load Neural Networks weights from memory if a valid checkpoint path is
    # passed
    if(policy_weights_path_1 != ""):
        print("Loading weights 1 from memory...")
        policy_network_1.load_weights(policy_weights_path_1)
    else:
        print("The networks path are empty.")
    #     sys.exit()

    if source == 3:
        state = acquire_frame(video_stream_url)
        time.sleep(1)
    state = _last_image
    # Save first image
    image_path = root_images + "image_0.png"
    cv2.imwrite(image_path, state)
    # 1 - Create a stack of X images
    image_t = np.stack([state] * images_stack_size, axis=2)
    matplotlib.interactive(True)
    # fig = plt.figure()
    f, (ax3, ax2) = plt.subplots(2, 1)
    f.tight_layout()
    # gs = gridspec.GridSpec(2, 2, width_ratios=[1, 1])
    # gs.update(left=0.05, right=0.95, wspace=0.05, hspace=0)
    # fig.set_size_inches(8, 4)
    f.patch.set_facecolor('gray')

    for step in range(1, 100000000):
        ##########################################
        ##                CAMERA                ##
        ##########################################
        pad_size = 1
        pad_value = 0
        # print "Shape image_t:" + str(np.shape(image_t))
        image = np.lib.pad(image_t[:, :, 0], (pad_size, pad_size),
                           'constant', constant_values=(pad_value, pad_value))
        for d in range(1, images_stack_size):
            image_stack_padded = np.lib.pad(
                image_t[:, :, d], (pad_size, pad_size), 'constant', constant_values=(pad_value, pad_value))
            image = np.append(image, image_stack_padded, axis=1)
        # print"shape: " + str(np.shape(image))
        # Plot in the first row the camera images
        # ax1 = plt.subplot(2, 1, 1)
        # ax1 = plt.subplot(gs[0, :])
        # but first clear the old one
        # ax1.clear()
        # ax1.axis("off")
        # specify greyscale instead BGR
        # ax1.imshow(image, cmap='gray')
        #########################################

        # 2 - Forward in input to NN
        # action_distribution_1 = policy_network_1.return_action_distribution(
        # input_data=np.reshape(image_t, (1, 84, 84, images_stack_size)),
        # softmax=False)
        action_distribution_1, conv1, conv2, conv3 = policy_network_1.return_everything(
            input_data=np.reshape(image_t, (1, 84, 84, images_stack_size)), softmax=False)
        action = np.argmax(action_distribution_1)
        action = convert_action_int_to_str(action)
        print "######################"
        print "Action distribution: " + str(action_distribution_1)
        print "Action_1 selected: " + action
        # print "Shape action dis:" + str(np.shape(action_distribution_1))
        # print "Shape conv1:" + str(np.shape(conv1))
        # print "Shape conv2:" + str(np.shape(conv2))
        # print "Shape conv3:" + str(np.shape(conv3))

        ##########################################
        ##                FILTERS               ##
        ##########################################
        ax3.clear()
        ax3.axis("off")

        padding = np.ones((1, 21 * 16))
        # 1 Conv layer ###########################
        conv1 = np.reshape(conv1, (21, 21, 32))
        image_cv1_1 = np.reshape(conv1[:, :, 0:16], (21, 21 * 16), order='F')
        image_cv1_2 = np.reshape(conv1[:, :, 16:32], (21, 21 * 16), order='F')
        # image_cv1_1 = np.reshape(conv1[:, :, 0:8], (21, 21 * 8), order='F')
        # image_cv1_2 = np.reshape(conv1[:, :, 8:16], (21, 21 * 8), order='F')
        # image_cv1_3 = np.reshape(conv1[:, :, 16:24], (21, 21 * 8), order='F')
        # image_cv1_4 = np.reshape(conv1[:, :, 24:32], (21, 21 * 8), order='F')
        image_cv1 = np.concatenate(
            (padding, image_cv1_1, image_cv1_2), axis=0)
        # Save filters
        filter_path = root_images + "filters/step_" + \
            str(step) + "/"
        if not os.path.exists(filter_path):
            os.makedirs(filter_path)
        filter_path = filter_path + "conv_1.jpg"

        I = image_cv1
        I8 = (((I - I.min()) / (I.max() - I.min())) * 255.9).astype(np.uint8)
        image_cv1_resized = cv2.resize(I8, (84 * 4, 21 * 2),
                                       interpolation=cv2.INTER_AREA)
        img = Image.fromarray(image_cv1_resized)
        img.save(filter_path)

        # 2 Conv layer ###########################
        padding = np.zeros((1, 11 * 32))
        conv2 = np.reshape(conv2, (11, 11, 64))
        image_cv2_1 = np.reshape(conv2[:, :, 0:32], (11, 11 * 32), order='F')
        image_cv2_2 = np.reshape(conv2[:, :, 32:64], (11, 11 * 32), order='F')
        # image_cv2_3 = np.reshape(conv2[:, :, 16:24], (21, 21 * 8), order='F')
        # image_cv2_4 = np.reshape(conv2[:, :, 24:32], (21, 21 * 8), order='F')
        image_cv2 = np.concatenate(
            (padding, image_cv2_1, image_cv2_2), axis=0)
        # Save filters
        filter_path = root_images + "filters/step_" + \
            str(step) + "/"
        if not os.path.exists(filter_path):
            os.makedirs(filter_path)
        filter_path = filter_path + "conv_2.jpg"

        I = image_cv2
        I8 = (((I - I.min()) / (I.max() - I.min())) * 255.9).astype(np.uint8)
        image_cv2_resized = cv2.resize(I8, (84 * 4, 11 * 2),
                                       interpolation=cv2.INTER_AREA)
        img = Image.fromarray(image_cv2_resized)
        img.save(filter_path)

        # 3 Conv layer ###########################
        padding = np.ones((1, 11 * 32))
        conv3 = np.reshape(conv3, (11, 11, 64))
        image_cv3_1 = np.reshape(conv3[:, :, 0:32], (11, 11 * 32), order='F')
        image_cv3_2 = np.reshape(conv3[:, :, 32:64], (11, 11 * 32), order='F')
        # image_cv2_3 = np.reshape(conv2[:, :, 16:24], (21, 21 * 8), order='F')
        # image_cv2_4 = np.reshape(conv2[:, :, 24:32], (21, 21 * 8), order='F')
        image_cv3 = np.concatenate(
            (padding, image_cv3_1, image_cv3_2), axis=0)
        # Save filters
        filter_path = root_images + "filters/step_" + \
            str(step) + "/"
        if not os.path.exists(filter_path):
            os.makedirs(filter_path)
        filter_path = filter_path + "conv_3.jpg"

        I = image_cv3
        I8 = (((I - I.min()) / (I.max() - I.min())) * 255.9).astype(np.uint8)
        image_cv3_resized = cv2.resize(I8, (84 * 4, 11 * 2),
                                       interpolation=cv2.INTER_AREA)
        img = Image.fromarray(image_cv3_resized)
        img.save(filter_path)

        # Assemble final image ####################

        image_input = np.reshape(image_t, (84, 84 * 4), order='F')
        image_input_resized = cv2.resize(image, (84 * 4, 84),
                                         interpolation=cv2.INTER_AREA)
        final_image = np.concatenate(
            (image_input_resized, image_cv1_resized, image_cv2_resized, image_cv3_resized))

        # Plot the image
        # ax3 = plt.subplot(gs[0, :])

        # but first clear the old one

        # specify greyscale instead BGR
        ax3 = plt.subplot(2, 1, 1)
        ax3.imshow(final_image, cmap='gray')
        ##########################################
        ##               HISTOGRAM              ##
        ##########################################
        # print np.shape(action_distribution_1)
        # print len(action_distribution_1[0])
        ind = np.arange(len(action_distribution_1[0]))
        # print ind
        width = 0.4
        # fig, ax = plt.subplots()
        # ax2 = plt.subplot(gs[1, :])
        ax2.clear()
        ax2 = plt.subplot(2, 1, 2)
        ax2.set_aspect(3)

        rects = ax2.bar(0.4 + ind, action_distribution_1[0],
                        width=width, color='r', alpha=0.4)
        rects[np.argmax(action_distribution_1)].set_color('g')
        ax2.set_xticks(0.4 + ind + width / 2)
        ax2.set_xticklabels(("left", "right", "forward",
                             "backward", "stop", "descend"))
        for i in ax2.xaxis.get_ticklabels():
            i.set_color("white")

        ax2.set_ylim([0, 1])

        # plt.xticks(fontsize=16)
        # plt.show()

        ##########################################

        # 3 - Send command and wait for translation
        raw_input("Press a button to acquire images...")

        # 4 - Acquire second frame and add to the stack
        if source == 3:
            global _last_image
            _last_image = acquire_frame(video_stream_url)
            time.sleep(1)
        image_t1 = _last_image
        # Save every one image image
        image_path = root_images + "image_" + str(step) + ".png"
        cv2.imwrite(image_path, image_t1)
        image_t1 = np.expand_dims(image_t1, 2)
        image_t1 = np.append(image_t[:, :, 1:], image_t1, axis=2)

        # 5 - Forward stack in input to the network
        image_t = image_t1


if __name__ == "__main__":
    # t1 = threading.Thread(name="VideoStream", target=acquire_frame)
    # t1.start()
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
