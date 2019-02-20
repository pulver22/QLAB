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
    replay_buffer_path = "/media/pulver/Pulver HD/replay_buffer.pickle"
    #replay_buffer_path = "./replay_buffer.pickle"
    replay_buffer = ExperienceReplayBuffer(capacity=replay_memory_size)
    
    timer_start = time.time()
    # Load the Replay buffer from file or accumulate experiences
    replay_buffer.load(replay_buffer_path)
    timer_stop = time.time()

    print "Time episode: " + str(timer_stop - timer_start) + " seconds"                
    print "Time episode: " + str((timer_stop - timer_start) / 60) + " minutes"
    print "Size : " + str(replay_buffer.return_size())
    print("")
		
    
    #----------------- PRINT EXPERIENCE FOR TESTING-----------------
    
    index = 97000
    increment = 1
    image, image_t1 = replay_buffer.debug_experience(index=index)
    image_t2, image_t3 = replay_buffer.debug_experience(index=index+increment)
    image_t4, image_t5 = replay_buffer.debug_experience(index=index+increment*2)
    image_t6, image_t7 = replay_buffer.debug_experience(index=index+increment*3)
    image_t8, image_t9 = replay_buffer.debug_experience(index=index+increment*4)
    image_vstack = np.vstack((image, image_t1, image_t2, image_t3, image_t4, image_t5, image_t6, image_t7, image_t8, image_t9))
    cv2.imshow("experience 0-9 vertical stack", image_vstack)

    index = 98000
    increment = 1
    image, image_t1 = replay_buffer.debug_experience(index=index)
    image_t2, image_t3 = replay_buffer.debug_experience(index=index+increment)
    image_t4, image_t5 = replay_buffer.debug_experience(index=index+increment*2)
    image_t6, image_t7 = replay_buffer.debug_experience(index=index+increment*3)
    image_t8, image_t9 = replay_buffer.debug_experience(index=index+increment*4)
    image_vstack = np.vstack((image, image_t1, image_t2, image_t3, image_t4, image_t5, image_t6, image_t7, image_t8, image_t9))
    cv2.imshow("experience 0-9 vertical stack - buffer 2", image_vstack)
    while True:    
	if cv2.waitKey(33) == ord('q'):
	    cv2.destroyAllWindows()
	    break
    
    #----------------------------------------------------------------


if __name__ == "__main__":
    main()
