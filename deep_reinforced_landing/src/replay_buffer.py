#!/usr/bin/env python
import numpy as np

class ReplayBuffer(object):

    def __init__(self, capacity, image_shape):
        self.capacity = capacity
        #self.image_t_array = np.zeros((capacity, image_shape[0], image_shape[1], image_shape[2]))
        #self.action_t_array = np.zeros((capacity, 1), dtype=np.int32)
        #self.reward_t_array = np.zeros((capacity, 1), dtype=np.float32)
        #self.image_t1_array = np.zeros((capacity, image_shape[0], image_shape[1], image_shape[2]))
        #self.done_t1_array = np.zeros((capacity, 1), dtype=np.bool)
        #TODO: is empty faster than zeros?
        self.image_t_array = np.empty((capacity, image_shape[0], image_shape[1], image_shape[2]))
        self.action_t_array = np.empty((capacity, 1), dtype=np.int32)
        self.reward_t_array = np.empty((capacity, 1), dtype=np.float32)
        self.image_t1_array = np.empty((capacity, image_shape[0], image_shape[1], image_shape[2]))
        self.done_t1_array = np.empty((capacity, 1), dtype=np.bool)
        self.num_experiences = 0
        self.rewrite_counter = 0

    def return_experience_batch(self, batch_size):
        # Randomly sample batch_size examples
        if self.num_experiences < batch_size:
            index_array = np.random.choice(self.num_experiences, self.num_experiences)
        else:
            index_array = np.random.choice(self.num_experiences, batch_size)
        #Take elements from the arrays
        #using axis=0 returns image batch of shape (batch_size, h, w, d)
        image_t_batch = np.take(self.image_t_array, index_array, axis=0)
        action_t_batch = np.take(self.action_t_array, index_array, axis=0)
        reward_t_batch = np.take(self.reward_t_array, index_array, axis=0)
        image_t1_batch = np.take(self.image_t1_array, index_array, axis=0)
        done_t1_batch = np.take(self.done_t1_array, index_array, axis=0)
        return [image_t_batch, action_t_batch, reward_t_batch, image_t1_batch, done_t1_batch]

    #TODO check if [:] is faster than .copy()
    def add_experience(self, state, action, reward, new_state, done):
        if self.num_experiences < self.capacity:
            #if the buffer is not full it appends the experience
            self.image_t_array[self.num_experiences][:] = state
            self.action_t_array[self.num_experiences][:] = action
            self.reward_t_array[self.num_experiences][:] = reward
            self.image_t1_array[self.num_experiences][:] = new_state
            self.done_t1_array[self.num_experiences][:] = done
            self.num_experiences += 1
        else:
            #If the buffer is full replace from the beginning
            self.image_t_array[self.rewrite_counter][:] = state
            self.action_t_array[self.rewrite_counter][:] = action
            self.reward_t_array[self.rewrite_counter][:] = reward
            self.image_t1_array[self.rewrite_counter][:] = new_state
            self.done_t1_array[self.rewrite_counter][:] = done
            self.rewrite_counter += 1
            if(self.rewrite_counter >= self.capacity): self.rewrite_counter = 0

    def return_size(self):
        # if buffer is full, return buffer size
        # otherwise, return experience counter
        return self.num_experiences

    def return_size_byte(self, value="byte"):
        if(value=="byte"): denominator = 1
        elif(value=="kilobyte"): denominator = 1024
        elif(value=="megabyte"): denominator = 1048576
        elif(value=="gigabyte"): denominator = 1073741824
        return (self.image_t_array.nbytes/denominator, self.action_t_array.nbytes/denominator, 
                self.reward_t_array.nbytes/denominator, self.image_t1_array.nbytes/denominator, self.done_t1_array.nbytes/denominator)

    def save(self, file_name="./replay_buffer.npz"):
        np.savez(file_name, self.image_t_array, self.action_t_array, 
                 self.reward_t_array, self.image_t1_array, self.done_t1_array,
                 self.num_experiences)

    def load(self, file_path):
        npzfile = np.load(file_path)
        self.image_t_array = npzfile['arr_0']
        self.action_t_array = npzfile['arr_1']
        self.reward_t_array = npzfile['arr_2']
        self.image_t1_array = npzfile['arr_3']
        self.done_t1_array = npzfile['arr_4']
        self.num_experiences = npzfile['arr_5'] 
        self.rewrite_counter = npzfile['arr_6']

