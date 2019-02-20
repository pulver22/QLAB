#!/usr/bin/env python

# The MIT License (MIT)
# Copyright (c) 2017 Riccardo Polvara
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# Create, fill and store a Experience Replay FIFO buffer in tensorflow.
# An experience is defined by the current state (a sequence of images acquired with the camea),
# the action take in this state, the reward and the next state
import time
import os
from experience_replay_buffer import ExperienceReplayBuffer


def main():
    replay_memory_size = 100000
    replay_buffer_path = "/home/pulver/Desktop/simulation_11/replay_buffer_negative_sand_laptop.pickle"
    replay_buffer = ExperienceReplayBuffer(capacity=replay_memory_size)
    if(os.path.isfile(replay_buffer_path) == True):
        print("Replay buffer loading from file: " + str(replay_buffer_path))
        replay_buffer.load(replay_buffer_path)
        print "Size: " + str(replay_buffer.return_size())
    else:
        print "Buffer replay does not exist!"

    #Load the second replay buffer
    replay_buffer_positive_path = "/home/pulver/Desktop/simulation_11/replay_buffer_negative_sand.pickle"
    replay_buffer_positive = ExperienceReplayBuffer(capacity=replay_memory_size)
    if(os.path.isfile(replay_buffer_positive_path) == True):
        print("Replay buffer loading from file: " + str(replay_buffer_positive_path))
        replay_buffer_positive.load(replay_buffer_positive_path)
        # Merge the two replay buffers in the first
        replay_buffer.append_from_file(replay_buffer_positive_path) #TODO: uncomment
        # Load the second buffer into the first one
        # replay_buffer.append(replay_buffer_positive, starting_point=0) # TODO: comment to enable the load from path
        print "Merge done! New size: " + str(replay_buffer.return_size())
        print "Saving the replay buffer in: " + replay_buffer_path
        print "Sit back and relax, it may take a while..."
        replay_buffer.save(replay_buffer_path)
        print "Done!"
    else:
        print "Second replay buffer does not exist!"

    print("Start merging...")
    # Merge the two replay buffers in the first
    replay_buffer.append_from_file(replay_buffer_positive_path) #TODO: uncomment
    # Load the second buffer into the first one
    #replay_buffer.append(replay_buffer_positive, starting_point=0) # TODO: comment to enable the load from path
    print "Merge done! New size: " + str(replay_buffer.return_size())
    print "Saving the replay buffer in: " + replay_buffer_path
    print "Sit back and relax, it may take a while..."
        eplay_buffer.save(replay_buffer_path)
    print "Done!"

if __name__ == "__main__":
    main()
