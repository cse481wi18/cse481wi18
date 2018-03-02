#! /usr/bin/env python

import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

VOICE = 'voice_kal_diphone'

class RobotSound(object):
    def __init__(self):
        self._soundhandle = SoundClient(blocking=True)
        while not rospy.is_shutdown() and not self._soundhandle.actionclient.wait_for_server(rospy.Duration(1.0)):
            rospy.logwarn('Waiting for sound play node...')
        rospy.loginfo('Sound play ready')

    def say(self, text):
        self._soundhandle.say(text, VOICE)

    def play_sound(self, sound_file):
        self._soundhandle.playWaveFromPkg('fetch_api', 'sounds/{}'.format(sound_file))

