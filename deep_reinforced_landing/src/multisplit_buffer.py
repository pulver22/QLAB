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

    #TODO: Variables to change based on paths and buffer size
    to_keep_original_experiences = 370664 #remove with a pop right the experiences in the original buffer (useful if corrupted)
    sub_buffer_size = 37066 #size of the sub-buffer created
    replay_memory_size = 500000
    root_path = "/media/pulver/6a87c355-c51f-4e43-9527-f0c5bd730ec4/pulver/Desktop/simulation_11/negative/"
    replay_buffer_path = "/media/pulver/6a87c355-c51f-4e43-9527-f0c5bd730ec4/pulver/Desktop/simulation_11/negative/replay_buffer_negative_big_shuffled_370664.pickle"

    #Load the main buffer
    replay_buffer = ExperienceReplayBuffer(capacity=replay_memory_size)
    if(os.path.isfile(replay_buffer_path) == True):
        print("Replay buffer loading from file: " + str(replay_buffer_path))
        replay_buffer.load(replay_buffer_path)
        print "Size: " + str(replay_buffer.return_size())
    else:
        print "Buffer replay does not exist!"

    # print("Starting pruning experiences...")
    # print("Initial size ..... " + str(replay_buffer.return_size()))
    # to_prune = replay_buffer.return_size() - to_keep_original_experiences
    # replay_buffer.pop_experience(to_prune, side="right")
    # print("New size ..... " + str(replay_buffer.return_size()))
    # print("Done!")

    print("Starting shuffling experiences...")
    replay_buffer.shuffle()
    print("Done!")

    print("Starting multisplit...")
    print "Sit back and relax, it may take a while..."
    multibuffer_list = [(root_path + "replay_buffer_negative_1.pickle", sub_buffer_size), (root_path + "replay_buffer_negative_2.pickle", sub_buffer_size), (root_path + "replay_buffer_negative_3.pickle", sub_buffer_size),
                        (root_path + "replay_buffer_negative_4.pickle", sub_buffer_size), (root_path + "replay_buffer_negative_5.pickle", sub_buffer_size), (root_path + "replay_buffer_negative_6.pickle", sub_buffer_size),
                        (root_path + "replay_buffer_negative_7.pickle", sub_buffer_size), (root_path + "replay_buffer_negative_8.pickle", sub_buffer_size), (root_path + "replay_buffer_negative_9.pickle", sub_buffer_size),
                        (root_path + "replay_buffer_negative_10.pickle", sub_buffer_size)]
    replay_buffer.multisplit(multibuffer_list)
    print("Done!")


if __name__ == "__main__":
    main()
