#! /usr/bin/env python

import fetch_api
import rospy

def main():
   # See the fetch_api/sounds folder for more sounds.
   rospy.init_node('sound_demo')
   robot_sound = fetch_api.RobotSound()
   robot_sound.say('Playing some sounds')
   robot_sound.say('Okay')
   robot_sound.play_sound('E04.wav')
   robot_sound.say('Trill')
   robot_sound.play_sound('E11.wav')
   robot_sound.say('Excuse me')
   robot_sound.play_sound('IExcuseMe1.wav')
   robot_sound.say('Wheeee')
   robot_sound.play_sound('IMmm1.wav')

if __name__ == '__main__':
    main()
