#!/usr/bin/env python

# The MIT License (MIT)
# Copyright (c) 2017 Riccardo Polvara
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
import time
import os
from experience_replay_buffer import ExperienceReplayBuffer


def main():

    #Load the main buffer
    replay_memory_size = 500000
    replay_buffer_path = "/home/pulver/Desktop/marine_descending/replay_buffer_192k.pickle"
    replay_buffer = ExperienceReplayBuffer(capacity=replay_memory_size)
    if(os.path.isfile(replay_buffer_path) == True):
        print("Replay buffer loading from file: " + str(replay_buffer_path))
        replay_buffer.load(replay_buffer_path)
        print "Size: " + str(replay_buffer.return_size())
    else:
        print "Buffer replay does not exist!"

    #Here we load all the mini-buffer needed later
    print("Loading the buffer list...")
    max_capacity = 200000
    path = "/home/pulver/Desktop/marine_descending/replay_buffer_194k.pickle"
    buffer_1 = ExperienceReplayBuffer(capacity=max_capacity)
    buffer_1.load(path)
    print "Size: " + str(buffer_1.return_size())

    # max_capacity = 100000
    # path = "/home/pulver/Desktop/marine_descending/replay_buffer_negative_8910.pickle"
    # buffer_2 = ExperienceReplayBuffer(capacity=max_capacity)
    # buffer_2.load(path)
    # print "Size: " + str(buffer_2.return_size())

    # max_capacity = 100000
    # path = "/home/pulver/Desktop/simulation_11/soil/replay_buffer_negative_soil.pickle"
    # buffer_3 = ExperienceReplayBuffer(capacity=max_capacity)
    # buffer_3.load(path)
    # print "Size: " + str(buffer_3.return_size())

    # max_capacity = 100000
    # path = "/home/pulver/Desktop/simulation_11/snow/replay_buffer_negative_snow.pickle"
    # buffer_4 = ExperienceReplayBuffer(capacity=max_capacity)
    # buffer_4.load(path)
    # print "Size: " + str(buffer_4.return_size())

    #max_capacity = 100000
    #path = "/home/pulver/Desktop/simulation_11/snow/replay_buffer_positive_snow_rotated_78200.pickle"
    #buffer_5 = ExperienceReplayBuffer(capacity=max_capacity)
    #buffer_5.load(path)

    #Here we append the file to the main buffer
    print("Starting multiappend...")
    print "Sit back and relax, it may take a while..."
    multibuffer_list = [buffer_1]#, buffer_2]#, buffer_3, buffer_4]
    replay_buffer.multiappend(multibuffer_list)
    print "New size: " + str(replay_buffer.return_size())
    print("Done!")

    #In the end we shuffle and save
    print("Starting shuffling experiences...")
    replay_buffer.shuffle()
    print("Done!")

    print("Starting save buffer...")
    replay_buffer.save("/home/pulver/Desktop/marine_descending/replay_buffer_neutral_final.pickle")
    print("Done!")


if __name__ == "__main__":
    main()
