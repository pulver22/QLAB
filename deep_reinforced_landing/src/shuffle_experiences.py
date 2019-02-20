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
import time

from experience_replay_buffer import ExperienceReplayBuffer

def main():
    
    replay_memory_size = 400000
    replay_buffer_path_1 = "/home/pulver/replay_2.pickle"
    
    #replay_buffer_path = "./replay_buffer.pickle"
    replay_buffer_1 = ExperienceReplayBuffer(capacity=replay_memory_size)

    
    timer_start = time.time()
    # Load the Replay buffer from file or accumulate experiences
    replay_buffer_1.load(replay_buffer_path_1)
    timer_stop = time.time()

    print "Time episode: " + str(timer_stop - timer_start) + " seconds"                
    print "Time episode: " + str((timer_stop - timer_start) / 60) + " minutes"
    print "Size_1 : " + str(replay_buffer_1.return_size())
    print("")

    print "Starting shuffling..."
    replay_buffer_1.shuffle()
    print "Shuffle completed!"
    print "Starting saving..."
    replay_buffer_1.save("/home/pulver/replay_2.pickle")

    print "Size_1 : " + str(replay_buffer_1.return_size())
if __name__ == "__main__":
    main()
