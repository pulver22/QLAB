#!/usr/bin/env python

# The MIT License (MIT)
# Copyright (c) 2017 Massimiliano Patacchiola
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
# FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
# OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
# 
# Implementation of a Experience Replay FIFO buffer in Python.
# Here I use the Tensorflow object RandomShuffleQueue which can store
# tuple and return random elements from the buffer.
# More details about the Experience Replay are available in:
# http://www.nature.com/nature/journal/v518/n7540/full/nature14236.html

from collections import deque
import random
try:
    import cpickle as pickle
except:
    import pickle
import sys
import numpy as np


class ExperienceReplayBuffer:
    """Class ExperienceReplayBuffer

    Implementation of the experience replay FIFO buffer for DQN.
    This class has methods for adding and getting experiences.
    It is based on Python deque class. 
    """

    def __init__(self, capacity):
        """Initialise the experience buffer.

        The initialisation phase requires to define the capacity of the buffer,
        the image shape and the number of frames stored (typically: 3-5).
        @param capacity it is an integer specifying the dimension of the buffer
        """
        if capacity <= 0:
            raise ValueError("[REPLAY BUFFER][ERROR] the capacity must be > 0")
        # to store: image_t, action_t, reward_t, image_t1, done_t1
        self.buffer = deque()
        self.capacity = capacity
        self.size = 0

    def add_experience(self, image_t, action_t, reward_t, image_t1, done_t1):
        """Add a new experience in the buffer

        The components of the experience are stored as tuple 
        (instead of lists) because they are immutable. It saves memory.
        @param image_t the image at time t
        @param action_t taken at time t
        @param reward_t obtained at time t
        @param image_t1 at time t+1
        @param done_t1 boolean indicating if t+1 is terminal
        """
        if self.size < self.capacity:
            self.buffer.append([image_t, action_t, reward_t, image_t1, done_t1])
            self.size += 1
        else:
            self.buffer.popleft()  # remove the left most item
            self.buffer.append((image_t, action_t, reward_t, image_t1, done_t1))

    def return_experience_batch(self, batch_size):
        """Return a batch_size-lenght list of experiences

        The list returned by this function is arranged as follow:
        First_element:  (image_t, action_t, reward_t, image_t1 done_t1)
        Second element: (image_t, action_t, reward_t, image_t1 done_t1)
        Third element:  (image_t, action_t, reward_t, image_t1 done_t1)
        ...
        @param batch_size an integer representing the number of 
            experiences to return
        @return a batch of experiences
        """
        if batch_size > self.return_size():
            raise Exception("ERROR: a batch of experience can be returned only if n < buffer_size")
        batch_experience = random.sample(self.buffer, batch_size)
        return batch_experience

    def return_size(self):
        """Return the number of elements inside the buffer

        @return an integer representing the number of elements
        """
        return self.size

    def return_size_byte(self, value='byte'):
        """Return the number bytes occupied by the buffer in memory

        @param value a string representing the type of value
            it can be: byte, kilobyte, megabyte, gigabyte.
        @return an integer representing the number of bytes
        """
        cumulated_size = 0
        cumulated_size += self.buffer[0][0].nbytes
        cumulated_size += sys.getsizeof(self.buffer[0][1])
        cumulated_size += sys.getsizeof(self.buffer[0][2])
        cumulated_size += self.buffer[0][3].nbytes
        cumulated_size += sys.getsizeof(self.buffer[0][4])
        cumulated_size = cumulated_size * len(self.buffer)
        if value == 'byte':
            return cumulated_size/1024
        elif value == 'kilobyte':
            return float(cumulated_size/1024)
        elif value == 'megabyte':
            return float(cumulated_size/1048576)
        elif value == 'gigabyte':
            return float(cumulated_size/1073741824)
        else: 
            raise Exception("[EXPERIENCE REPLAY BUFFER] Error: "
                            "the value must be one of (byte, kilobyte, megabyte, gigabyte)")

    def save(self, file_name):
        """ Save the buffer in a file.

        @param file_name
        """
        with open(file_name, 'wb') as f:
            pickle.dump(self.buffer, f, protocol=pickle.HIGHEST_PROTOCOL)

    def load(self, file_name):
        """Load the buffer from a file.

        @param file_name
        """
        self.buffer = pickle.load(open(file_name, 'rb'))
        self.size = len(self.buffer)

    def debug_experience(self, index=None, pad_size=2, pad_value=0, print_info=True):
        """ Return a vertical stack of the images contained in the experience and print experience values.
        
        The experience should contain grayscale images aligned in depth dimension.
        The function returns a vertical stack of images and print on terminal the reward, action, done values.
        @param index: the index of the image to return, if None a random experience is returned
        @para pad_size: the number of values to use for padding the images (default 2 pixels)
        @param pad_value: the value to use for the padding (default 0 = black in OpenCV)
        @param print_info: when True print the values of action, reward, done (default True)
        @return: a vertical (padded with zeros) stack of images contained in the experience
            if there is an anomaly in the images (wrong depth) return None,None
        """

        if index is None:
            index = random.randint(0, len(self.buffer)-1)
        else:
            pass
        # (image_t, action_t, reward_t, image_t1, done_t1)
        image_stack = self.buffer[index][0]
        action = self.buffer[index][1]
        reward = self.buffer[index][2]
        image_t1_stack = self.buffer[index][3]
        done = self.buffer[index][4]
        if print_info:
            print("Index  ..... " + str(index))
            print("Action ..... " + str(action))
            print("Reward ..... " + str(reward))
            print("Done   ..... " + str(done))
            print("")
        if len(image_stack.shape) == 2:
            return np.lib.pad(image_stack, (pad_size, pad_size), 'constant', constant_values=(pad_value, pad_value)), np.lib.pad(image_t1_stack, (pad_size, pad_size), 'constant', constant_values=(pad_value, pad_value))
        elif len(image_stack.shape) == 3:
            image = np.lib.pad(image_stack[:, :, 0], (pad_size, pad_size), 'constant', constant_values=(pad_value, pad_value))
            image_t1 = np.lib.pad(image_t1_stack[:, :, 0], (pad_size, pad_size), 'constant', constant_values=(pad_value, pad_value))
            depth = image_stack.shape[2]
            for d in range(1, depth):
                image_stack_padded = np.lib.pad(image_stack[:, :, d], (pad_size, pad_size), 'constant', constant_values=(pad_value, pad_value))
                image_t1_stack_padded = np.lib.pad(image_t1_stack[:, :, d], (pad_size, pad_size), 'constant',
                                                constant_values=(pad_value, pad_value))
                image = np.append(image, image_stack_padded, axis=1)
                image_t1 = np.append(image_t1, image_t1_stack_padded, axis=1)
            return image, image_t1
        else:
            return None, None


    def append(self, replay_buffer_2):
        """
        Append a second replay buffer at the end of another one, if there is available space

        @param replay_buffer_2 is a second replay buffer
        """
        for i in range(0, len(replay_buffer_2.buffer)):
            element = replay_buffer_2.buffer.pop()
            #print len(element)
            self.add_experience(element[0], element[1], element[2], element[3], element[4])

    def count_experience(self):
        """
        Count the number of positive, neutral and negative experiences contained in a buffer.
        """
        counter_positive = 0
        counter_neutral = 0
        counter_negative = 0

        for i in self.buffer:
            if i[2] == 1.0:
                counter_positive += 1
            elif i[2] == -1.0:
                counter_negative += 1
            else:
                counter_neutral += 1
        
        print "Number of positive experiences: " + str(counter_positive)
        print "Number of negative experiences: " + str(counter_negative)
        print "Number of neutral experiences: " + str(counter_neutral)