import time
import os
from experience_replay_buffer import ExperienceReplayBuffer
import numpy as np
import cv2

def main():
    replay_memory_size = 100000
    replay_buffer_path = "/home/pulver/Desktop/simulation_11/sand/replay_buffer_positive_sand.pickle"
    replay_buffer = ExperienceReplayBuffer(capacity=replay_memory_size)
    if(os.path.isfile(replay_buffer_path) == True):
        print("Replay buffer loading from file: " + str(replay_buffer_path))
        replay_buffer.load(replay_buffer_path)
        print "Size: " + str(replay_buffer.return_size())
    else:
        print "Buffer replay does not exist!"
        return

    # experience = replay_buffer.buffer[0]
    # image_t = experience[0]
    # action_t = experience[1]
    # reward_t = experience[2] 
    # image_t1 = experience[3] 
    # done_t1 = experience[4]

    # Modify the whole buffer or just add a single experience
    replay_buffer.rotate_and_push(rotations_list=['90', '180', '270', 'vflip', 'vflip90', 'vflip180', 'vflip270'], tot_elements=None)
    replay_buffer.shuffle()
    #replay_buffer.add_experience_and_rotate(image_t, action_t, reward_t, image_t1, done_t1, rotation_list=['90', '180', '270', 'vflip', 'vflip90', 'vflip180', 'vflip270'])
    print("Rotation done! New size: " + str(replay_buffer.return_size()))
    print("Saving the buffer...")
    replay_buffer.save(file_name="/home/pulver/Desktop/simulation_11/sand/replay_buffer_positive_sand_rotated_" + str(replay_buffer.return_size()) + ".pickle")
    print("Done!")

    # # Show the experiences
    # index = replay_buffer.return_size() - 8
    # # Original image
    # original_t, original_t1 = replay_buffer.debug_experience(index=index)
    # image_vstack = np.vstack((original_t, original_t1))
    # cv2.imshow("experiences original", image_vstack)
    # # Rotated images
    # image, image_t1 = replay_buffer.debug_experience(index=index+1)
    # image_t2, image_t3 = replay_buffer.debug_experience(index=index+2)
    # image_t4, image_t5 = replay_buffer.debug_experience(index=index+3)
    # image_t6, image_t7 = replay_buffer.debug_experience(index=index+4)
    # # Shifted and rotated
    # image_t8, image_t9 = replay_buffer.debug_experience(index=index+5)
    # image_t10, image_t11 = replay_buffer.debug_experience(index=index+6)
    # image_t12, image_t13 = replay_buffer.debug_experience(index=index+7)
    # image_vstack_rotated = np.vstack((image, image_t1, image_t2, image_t3, image_t4, image_t5))
    # image_vstack_flipped = np.vstack((image_t6, image_t7, image_t8, image_t9, image_t10, image_t11, image_t12, image_t13))
    # # Rotated and flipped images
    # cv2.imshow("experiences rotated", image_vstack_rotated)
    # cv2.imshow("experiences flipped", image_vstack_flipped)
    # while True:    
	#     if cv2.waitKey(33) == ord('q'):
	#         cv2.destroyAllWindows()
	#         break

if __name__ == "__main__":
    main()