#!/usr/bin/env python
import time
import rospy
import pygame
class qwer_print(object):
    def __init__(self):
        for i in range (3):
            rospy.loginfo('****************TEST*********** %f' % i)
        self.music()

    def music(self):
        sound = '/home/ubuntu/duckietown/catkin_ws/src/summer_school/qwer_nutn/socket_qwer/include/socket_qwer/Desp.ogg'
        pygame.mixer.init()
        pygame.mixer.music.load(sound)
        pygame.mixer.music.set_volume(0.05)  #The value argument is between 0.0 and 1.0.
        pygame.mixer.music.play(0, 0) #1st number set loops, 2nd controls where in the music the song starts playing.
        print('-------IN def music(self)-------------')
        while pygame.mixer.music.get_busy(): 
            pygame.time.Clock().tick(10)

if __name__ == '__main__':
    rospy.init_node("qwer_print",anonymous=False)
    node_print = qwer_print()
    rospy.spin()
