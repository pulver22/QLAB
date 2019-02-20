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
import numpy as np
import sys
from experience_replay_buffer import ExperienceReplayBuffer
import Image
import datetime
import time
import os.path
import cv2



def main():
    
    replay_memory_size = 400000
    replay_buffer_path = "/home/pulver/Desktop/test/replay_buffer_positive.pickle"
    replay_buffer_path_2 = "/home/pulver/Desktop/replay_buffer_positive_rick.pickle"
    #replay_buffer_path = "./replay_buffer.pickle"
    replay_buffer = ExperienceReplayBuffer(capacity=replay_memory_size)
    replay_buffer_2 = ExperienceReplayBuffer(capacity=replay_memory_size)
    # Save the empty buffer
    if(os.path.isfile(replay_buffer_path_2) == True):  
        print "The second buffer exist"
    else:
        print "Buffer not existing. I am saving an empty one."
        replay_buffer_2.save(replay_buffer_path_2)
    
    timer_start = time.time()
    # Load the Replay buffer from file or accumulate experiences
    replay_buffer.load(replay_buffer_path)
    timer_stop = time.time()
    
    replay_buffer.copy_experience_to(replay_buffer_2, replay_buffer_path_2, 2000)
    replay_buffer_2.load(replay_buffer_path_2)

    print "Time episode: " + str(timer_stop - timer_start) + " seconds"                
    print "Time episode: " + str((timer_stop - timer_start) / 60) + " minutes"
    print "Size : " + str(replay_buffer.return_size())
    print "Size 2: " + str(replay_buffer_2.return_size())
    print("")
		
    
    #----------------- PRINT EXPERIENCE FOR TESTING-----------------
    
    
    
    #----------------------------------------------------------------


if __name__ == "__main__":
    main()
