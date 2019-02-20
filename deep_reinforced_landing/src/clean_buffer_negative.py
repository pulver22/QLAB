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
from experience_replay_buffer import ExperienceReplayBuffer




def main():
    
    replay_memory_size = 400000
    replay_buffer_path = "/home/thebeast/Desktop/buffer_to_merge/negative/buffer_negative.pickle" 
    replay_buffer = ExperienceReplayBuffer(capacity=replay_memory_size)
    
    #timer_start = time.time()
    # Load the Replay buffer from file or accumulate experiences
    replay_buffer.load(replay_buffer_path)
    #timer_stop = time.time()
    original_size = replay_buffer.return_size()
    print "Original size: " + str(original_size)
    # counter_removed = replay_buffer.clean_action_reward("descend", -1.0)
    # print "New size: " + str(replay_buffer.return_size())
    # print "Number experiences removed:" + str(counter_removed)
    # print "Percentage removed experiences: " + str(100 * counter_removed/float(original_size)) + "%"
    # replay_buffer.save("./clean_buffer_positive.pickle")
    counter = replay_buffer.count_experiences("descend", -1.0)
    print "Tot wrond experiences: " + str(counter)

if __name__ == "__main__":
    main()
