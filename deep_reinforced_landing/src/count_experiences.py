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
    replay_buffer_path = "/home/pulver/Desktop/buffers_comparison/replay_buffer_shared.pickle"
    replay_buffer_positive_path = "/home/pulver/Desktop/buffers_comparison/replay_buffer_positive.pickle"
    replay_buffer_negative_path = "/home/pulver/Desktop/buffers_comparison/replay_buffer_negative.pickle"
    replay_buffer_neutral_path = "/home/pulver/Desktop/buffers_comparison/replay_buffer_neutral.pickle"
    #replay_buffer_path = "./replay_buffer.pickle"
    replay_buffer = ExperienceReplayBuffer(capacity=replay_memory_size)
    replay_buffer_positive = ExperienceReplayBuffer(capacity=replay_memory_size)
    replay_buffer_negative = ExperienceReplayBuffer(capacity=replay_memory_size)
    replay_buffer_neutral = ExperienceReplayBuffer(capacity=replay_memory_size)
    
    timer_start = time.time()
    # Load the Replay buffer from file or accumulate experiences
    replay_buffer.load(replay_buffer_path)
    replay_buffer_positive.load(replay_buffer_positive_path)
    replay_buffer_negative.load(replay_buffer_negative_path)
    replay_buffer_neutral.load(replay_buffer_neutral_path)
    timer_stop = time.time()

    print "Time episode: " + str(timer_stop - timer_start) + " seconds"                
    print "Time episode: " + str((timer_stop - timer_start) / 60) + " minutes"
    print "Size shared buffer: " + str(replay_buffer.return_size())
    print "Size positive buffer: " + str(replay_buffer_positive.return_size())
    print "Size negative buffer: " + str(replay_buffer_negative.return_size())
    print "Size neutral buffer: " + str(replay_buffer_neutral.return_size())
    print("")
		
    
    #----------------- PRINT EXPERIENCE COUNTER -----------------
    print "Shared buffer"
    replay_buffer.count_experience()
    print ""
    print "Positive buffer"
    replay_buffer_positive.count_experience()
    print ""    
    print "Negative buffer"
    replay_buffer_negative.count_experience()
    print ""
    print "Neutral buffer"
    replay_buffer_neutral.count_experience()
    print ""
    #----------------------------------------------------------------


if __name__ == "__main__":
    main()
