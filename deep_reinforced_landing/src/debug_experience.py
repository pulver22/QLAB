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
from deep_reinforced_landing.srv import NewCameraService, GetDoneAndReward, SendCommand, ResetPosition
import rospy
# Rename to avoid confusion with Image lib
from sensor_msgs.msg import Image as ROSImage


DEBUG = False  # Set to False to disable the image shown at the begining
_last_image = None  # Global variable representing the last frame acquired by the camera
_save_image = False


def main():
    """
    Main function for training the DQN network in learning how to accomplish autonomous landing.
    """
    

    timer_total_start = time.time()
    rospy.init_node("DeepReinforcedLanding")
    rospy.loginfo("----- Deep Reinforced Landing Node -----")
    replay_memory_size = 400000
    replay_buffer_path = "./replay_buffer_test.pickle"
    #replay_buffer_path = "./replay_buffer.pickle"
    replay_buffer = ExperienceReplayBuffer(capacity=replay_memory_size)




    # Load the Replay buffer from file or accumulate experiences

    replay_buffer.load(replay_buffer_path)
    
    #----------------- PRINT EXPERIENCE FOR TESTING-----------------
    
    index = 0
    increment = 1
    image, image_t1 = replay_buffer.debug_experience(index=index)
    image_t2, image_t3 = replay_buffer.debug_experience(index=index+increment)
    image_t4, image_t5 = replay_buffer.debug_experience(index=index+increment*2)
    image_t6, image_t7 = replay_buffer.debug_experience(index=index+increment*3)
    image_t8, image_t9 = replay_buffer.debug_experience(index=index+increment*4)
    image_vstack = np.vstack((image, image_t1, image_t2, image_t3, image_t4, image_t5, image_t6, image_t7, image_t8, image_t9))
    cv2.imshow("experience 0-9 vertical stack", image_vstack)
    while True:    
	if cv2.waitKey(33) == ord('q'):
	    cv2.destroyAllWindows()
	    break
    
    #----------------------------------------------------------------

if __name__ == "__main__":
    main()
