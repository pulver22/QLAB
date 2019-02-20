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
# Implementation of a Convolutional Neural Network (CNN) in tensorflow.
# This class can be used in DQN and is an implementation of a Q-Network.
# More details about the Q-Network are available in:
# http://www.nature.com/nature/journal/v518/n7540/full/nature14236.html

import tensorflow as tf

DEBUG = True


class QNetwork:
    """Class QNetwork

    Implementation of the Q-Network for Deep Reinforcement Learning.
    """

    def __init__(self, tf_session, tot_actions, image_shape, batch_size, network_name):
        """Initialise the Q-Network allocating the variables in memory.

        The initialisation phase requires to define the image shape 
        and the number of frames in input (typically: 3-5). The network has the
        following properties:
        -PADDING: type 'VALID' is not used because it may drop useful information.
             type 'SAME' because it does not drop the pixel on the border. Those
             pixels can be informative in Breakout when the ball hits the wall. 

        Important: after the initialisation of the object it is necessary to
        initialise all the variable calling the tf method tf.initialize_all_variables().
        @param tf_session the tensorflow session
        @param image_shape the shape in the form (rows, columns, frames) of the images
        @param network_name the name of the net (it will appear in the tf session)
        """

        # Section "Methods" from the paper: http://www.nature.com/nature/journal/v518/n7540/full/nature14236.html
        # The input to the neural network consists of an 84x84x4 image produced by the preprocessing map.
        # 1) The first hidden layer convolves 32 filters of 8x8 with stride 4 with the input image and applies a rectifier nonlinearity.
        # 2) The second hidden layer convolves 64 filters of 4x4 with stride 2, again followed by a rectifier nonlinearity.
        # 3) This is followed by a third convolutional layer that convolves 64 filters of 3x3 with stride 1 followed by a rectifier.
        # 4) The final hidden layer is fully-connected and consists of 512 rectifier units.
        # 5) The output layer is a fully-connected linear layer with a single output for each valid action.
        # The number of valid actions varied between 4 and 18 on the games we
        # considered.
        self.tf_session = tf_session
        self._num_labels = tot_actions
        self.network_name = network_name
        self.batch_size = batch_size
        self.image_h = image_shape[0]
        self.image_w = image_shape[1]
        self.image_depth = image_shape[2]
        self.tf_input_vector = tf.placeholder(tf.float32, shape=(
            None, self.image_h, self.image_w, self.image_depth))
        self.tf_batch_qvalue_target = tf.placeholder(
            tf.float32, shape=(batch_size))
        self.tf_batch_action_vector = tf.placeholder(
            tf.int32, shape=(batch_size))  # vector of action indeces
        self.tf_use_softmax = tf.placeholder(tf.int32)

        # Set the weights to random values
        # self._allocate_weights()
        # allocate the weights using an initiliazer (default=None=Glorot)
        self._allocate_weights_initializer()

        # Model. Defining the network model.
        def model(input_data, use_softmax=0):
            self.X = tf.reshape(
                input_data, shape=[-1, self.image_h, self.image_w, self.image_depth])
            self.X_normalised = tf.to_float(self.X) / 255.0
            if DEBUG:
                print("SHAPE X: " + str(self.X_normalised.get_shape()))
            # Convolution Layer 1
            # 1) The first hidden layer convolves 32 filters of 8x8 with stride 4 with the input image and applies a rectifier nonlinearity.
            # strides: A list of ints. 1-D of length 4. The stride of the
            # sliding window for each dimension of input.
            self.conv1 = tf.nn.relu(tf.nn.bias_add(tf.nn.conv2d(
                self.X_normalised, self.conv1_weights, strides=[1, 4, 4, 1], padding='SAME'), self.conv1_biases))
            if DEBUG:
                print("SHAPE conv1: " + str(self.conv1.get_shape()))
            # Convolution Layer 2
            # 2) The second hidden layer convolves 64 filters of 4x4 with
            # stride 2, again followed by a rectifier nonlinearity.
            self.conv2 = tf.nn.relu(tf.nn.bias_add(tf.nn.conv2d(
                self.conv1, self.conv2_weights, strides=[1, 2, 2, 1], padding='SAME'), self.conv2_biases))
            if DEBUG:
                print("SHAPE conv2: " + str(self.conv2.get_shape()))
            # Convolution Layer 3
            # 3) This is followed by a third convolutional layer that convolves
            # 64 filters of 3x3 with stride 1 followed by a rectifier.
            self.conv3 = tf.nn.relu(tf.nn.bias_add(tf.nn.conv2d(
                self.conv2, self.conv3_weights, strides=[1, 1, 1, 1], padding='SAME'), self.conv3_biases))
            if DEBUG:
                print("SHAPE conv3: " + str(self.conv3.get_shape()))
            # Fully-connected Layer 4
            # 4) The final hidden layer is fully-connected and consists of 512
            # rectifier units.
            dense1 = tf.reshape(
                self.conv3, [-1, self.dense1_weights.get_shape().as_list()[0]])  # Reshape conv3
            if DEBUG:
                print("SHAPE dense1: " + str(dense1.get_shape()))
            dense1 = tf.nn.relu(
                tf.matmul(dense1, self.dense1_weights) + self.dense1_biases)
            # Output layer 5
            # 5) The output layer is a fully-connected linear layer with a
            # single output for each valid action.
            out = tf.matmul(dense1, self.out_weights) + self.out_biases
            if DEBUG:
                print("SHAPE out: " + str(out.get_shape()))
            # The output is evaluated based on the use_softmax parameter
            # Here tf.cond can evaluate the value of a placeholder and based on this value it can decide what to do
            # out = tf.cond(use_softmax>0, lambda: tf.nn.softmax(tf.matmul(dense1, self.out_weights) + self.out_biases), lambda: tf.matmul(dense1, self.out_weights) + self.out_biases)
            out = tf.matmul(dense1, self.out_weights) + \
                self.out_biases  # No softmax used
            return out

        # Operation for getting the result from the model
        self.cnn_output = model(self.tf_input_vector,
                                use_softmax=self.tf_use_softmax)

        # Operation for learning and monitoring learning
        # Because the method tf.gather can select elements only on flat array
        # it is necessary to modify the 'self.tf_batch_action_vector' as
        # following
        indices = (tf.range(start=0, limit=self.batch_size, dtype=tf.int32)
                   * self._num_labels) + self.tf_batch_action_vector
        # From cnn_batch_output (batch_size, tot_actions) takes only the action-indices in self.tf_batch_action_vector (batch_size),
        # leading to the Q-Values in 'batch_qvalue_prediction' (batch_size)
        # used to minimise the cost.
        batch_qvalue_prediction = tf.gather(tf.reshape(
            self.cnn_output, [-1]), indices)  # shape (batch_size)
        # The agent must learn that these actions in these states lead to these
        # q-values
        self.cost = tf.reduce_mean(tf.squared_difference(
            self.tf_batch_qvalue_target, batch_qvalue_prediction))
        # In the original paper: tf.train.RMSPropOptimizer(0.00025, 0.99, 0.0,
        # 1e-6)
        self.optimizer = tf.train.RMSPropOptimizer(
            learning_rate=0.00025, decay=0.99, momentum=0.0, epsilon=1e-6).minimize(self.cost)
        self.rmse = tf.sqrt(tf.square(tf.reduce_mean(
            self.tf_batch_qvalue_target - batch_qvalue_prediction)))
        self.q_max = tf.reduce_max(self.cnn_output)  # the Q-Value max
        self.q_mean = tf.reduce_mean(self.cnn_output)  # the Q-Value max
        self.q_min = tf.reduce_min(self.cnn_output)  # the Q-Value max

        # Set the summaries
        self.summaries = tf.summary.merge([tf.summary.scalar("cost", self.cost),
                                           tf.summary.histogram(
                                               "q_values_hist", self.cnn_output),
                                           tf.summary.scalar(
                                               "q_max", self.q_max),
                                           tf.summary.scalar(
                                               "q_min", self.q_min),
                                           tf.summary.scalar("q_mean", self.q_mean)])
        # Set the summaries for the convolution and input images
        single_conv1_weight = tf.reshape(
            self.conv1[:, :, :, 0], (-1, 21, 21, 1))  # shape: (?, 21, 21, 32)
        input_features = tf.reshape(self.X[:, :, :, 0], (-1, 84, 84, 1))
        self.filter_summaries = tf.summary.merge([tf.summary.image("input", input_features, max_outputs=self.batch_size),
                                                  tf.summary.image("filters/conv1", single_conv1_weight, max_outputs=self.batch_size)])

    def return_name(self):
        """Return the name of the network.

        @return a string representing the name of the network
        """
        return self.network_name

    @staticmethod
    def _return_clipped_error(x):
        """ Return the clipped error.

        Taken from:
        https://github.com/devsisters/DQN-tensorflow/blob/master/dqn/ops.py
        @param x the value to clip
        @return the clipped value
        """
        try:
            return tf.select(tf.abs(x) < 1.0, 0.5 * tf.square(x), tf.abs(x) - 0.5)
        except:
            return tf.where(tf.abs(x) < 1.0, 0.5 * tf.square(x), tf.abs(x) - 0.5)

    def get_weights(self):
        """ Get a list containing the 10 weights vectors of the network.

        This function can be used to move the weights around and to assign
        them to another network (e.g. in the Q-Network reset step of DQN algorithm).
        @return the list containing the weights
        """
        weights_list = [self.conv1_weights,  self.conv1_biases,
                        self.conv2_weights,  self.conv2_biases,
                        self.conv3_weights,  self.conv3_biases,
                        self.dense1_weights, self.dense1_biases,
                        self.out_weights,    self.out_biases]
        return weights_list

    def _allocate_weights(self, std=0.01):
        """Allocate the wiehgts using a truncated normal distribution with STD=std.

        @param std the standard deviation of the normal curve
        """
        # Conv layer
        # 1) The first hidden layer convolves 32 filters of 8x8 with stride 4 with the input image and applies a rectifier nonlinearity.
        #[patch_size, patch_size, num_channels, depth]
        self.conv1_weights = tf.Variable(tf.truncated_normal(
            [8, 8, self.image_depth, 32], stddev=std), name=self.network_name + "_conv1_weights")
        self.conv1_biases = tf.Variable(
            tf.zeros([32]), name=self.network_name + "_conv1_biases")
        # Conv layer
        # 2) The second hidden layer convolves 64 filters of 4x4 with stride 2, again followed by a rectifier nonlinearity.
        #[patch_size, patch_size, depth, depth]
        self.conv2_weights = tf.Variable(tf.truncated_normal(
            [4, 4, 32, 64], stddev=std), name=self.network_name + "_conv2_weights")
        self.conv2_biases = tf.Variable(tf.random_normal(
            shape=[64]), name=self.network_name + "_conv2_biases")
        # Conv layer
        # 3) This is followed by a third convolutional layer that convolves 64
        # filters of 3x3 with stride 1 followed by a rectifier.
        self.conv3_weights = tf.Variable(tf.truncated_normal(
            [3, 3, 64, 64], stddev=std), name=self.network_name + "_conv3_weights")
        self.conv3_biases = tf.Variable(tf.random_normal(
            shape=[64]), name=self.network_name + "_conv3_biases")
        # Dense layer
        # 4) The final hidden layer is fully-connected and consists of 512 rectifier units.
        #[ 5*5 * previous_layer_out , num_hidden] wd1
        # here 5*5 is the size of the image after pool reduction (divide by half 3 times)
        # tot_maps = float(self.image_h/4)/2 #the first stride is 4 and the second is 2
        #tot_maps = int(math.ceil(tot_maps))
        self.dense1_weights = tf.Variable(tf.truncated_normal(
            [11 * 11 * 64, 512], stddev=std), name=self.network_name + "_dense1_weights")  # was [5*5*256, 1024]
        self.dense1_biases = tf.Variable(tf.random_normal(
            shape=[512]), name=self.network_name + "_dense1_biases")
        # Output layer
        # 5) The output layer is a fully-connected linear layer with a single
        # output for each valid action.
        self.out_weights = tf.Variable(tf.truncated_normal(
            [512, self._num_labels], stddev=std), name=self.network_name + "_out_weights")
        self.out_biases = tf.Variable(tf.random_normal(
            shape=[self._num_labels]), name=self.network_name + "_out_biases")

    def _allocate_weights_initializer(self, initializer=None):
        """ Allocate the wiehgts using a specific initializer.

        @param initializer the initializer to use.
              (default: None) > glorot_uniform_initializer
        """
        # Conv layer
        # 1) The first hidden layer convolves 32 filters of 8x8 with stride 4 with the input image and applies a rectifier nonlinearity.
        #[patch_size, patch_size, num_channels, depth]
        self.conv1_weights = tf.get_variable(self.network_name + "_conv1_weights", shape=[
                                             8, 8, self.image_depth, 32], initializer=initializer)
        self.conv1_biases = tf.get_variable(
            self.network_name + "_conv1_biases", shape=[32], initializer=tf.constant_initializer(0.0))
        # Conv layer
        # 2) The second hidden layer convolves 64 filters of 4x4 with stride 2, again followed by a rectifier nonlinearity.
        #[patch_size, patch_size, depth, depth]
        self.conv2_weights = tf.get_variable(
            self.network_name + "_conv2_weights", shape=[4, 4, 32, 64], initializer=initializer)
        self.conv2_biases = tf.get_variable(
            self.network_name + "_conv2_biases", shape=[64], initializer=tf.constant_initializer(0.0))
        # Conv layer
        # 3) This is followed by a third convolutional layer that convolves 64
        # filters of 3x3 with stride 1 followed by a rectifier.
        self.conv3_weights = tf.get_variable(
            self.network_name + "_conv3_weights", shape=[3, 3, 64, 64], initializer=initializer)
        self.conv3_biases = tf.get_variable(
            self.network_name + "_conv3_biases", shape=[64], initializer=tf.constant_initializer(0.0))
        # Dense layer
        # 4) The final hidden layer is fully-connected and consists of 512
        # rectifier units.
        self.dense1_weights = tf.get_variable(
            self.network_name + "_dense1_weights", shape=[11 * 11 * 64, 512], initializer=initializer)
        self.dense1_biases = tf.get_variable(
            self.network_name + "_dense1_biases", shape=[512], initializer=tf.constant_initializer(0.0))
        # Output layer
        # 5) The output layer is a fully-connected linear layer with a single
        # output for each valid action.
        self.out_weights = tf.get_variable(
            self.network_name + "_out_weights", shape=[512, self._num_labels], initializer=initializer)
        self.out_biases = tf.get_variable(self.network_name + "_out_biases", shape=[
                                          self._num_labels], initializer=tf.constant_initializer(0.0))

    def set_weights(self, copy_from_network):
        """
        Set the network weights taking it from the input network.

        @param copy_from_network the network to copy from
        """
        net1_params = [t for t in tf.trainable_variables(
        ) if t.name.startswith(self.network_name)]
        net2_params = [t for t in tf.trainable_variables(
        ) if t.name.startswith(copy_from_network.return_name())]
        # Sort by names
        net1_params = sorted(net1_params, key=lambda v: v.name)
        net2_params = sorted(net2_params, key=lambda v: v.name)
        # Get the ops
        update_ops = []
        for net1_v, net2_v in zip(net1_params, net2_params):
            op = net1_v.assign(net2_v)
            update_ops.append(op)
        # Run the session with the ops
        self.tf_session.run(update_ops)

    def save_weights(self, file_path):
        """ Save the weights of the network.

        @param file_path the path where the file is stored
        """
        # Add ops to save and restore all the variables.
        saver = tf.train.Saver()
        saver.save(self.tf_session, file_path)

    def load_weights(self, file_path):
        """ Load the weights of the network.

        Important: the weights must be loaded only after initialising
          a network which has the same name of the network stored previously.
          Moreover the tf method tf.initialize_all_variables() must be called
          before using the restore utility because it requires the weights to
          be allocated in memory.
        @param file_path the path where the file is stored
        """
        # Add ops to save and restore all the variables.
        saver = tf.train.Saver()
        saver.restore(self.tf_session, file_path)

    def return_action_distribution(self, input_data, softmax=False):
        """Return the network output based on a single input. It can be used
             to evaluate the argmax of the action to take at a given time step.

        The input must be of the right shape. 
        @param input_data a set of images of the same shape as declared in the init
        @param softmax if True applies the softmax to the output and
             returns a probability distribution on the action vector.
        """
        feed_dict = {self.tf_input_vector: input_data,
                     self.tf_use_softmax: int(softmax)}
        action_vector = self.tf_session.run(
            [self.cnn_output], feed_dict=feed_dict)
        return action_vector

    def perform_gradient_descent_step(self, input_data_batch, action_data_batch, target_data_batch):
        """ Perform one step of gradient descent.

        The input must be of the right shape. 
        @param input_data_batch a set of images of the same shape as declared in the init
        @param action_data_batch a set of actions taken for the input batch
        @param target_data_batch a batch of vectors corresponding to the Target, shape (batch_size, tot_actions)
        @return the loss value and the root mean squared error (RMSE)
        """
        feed_dict = {self.tf_input_vector: input_data_batch, self.tf_batch_action_vector: action_data_batch,
                     self.tf_batch_qvalue_target: target_data_batch, self.tf_use_softmax: 0}
        _optimizer, _summaries, _filter_summaries = self.tf_session.run(
            [self.optimizer, self.summaries, self.filter_summaries], feed_dict=feed_dict)
        return _summaries, _filter_summaries
