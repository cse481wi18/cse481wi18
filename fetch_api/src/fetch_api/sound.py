#! /usr/bin/env python

import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

VOICE = 'voice_kal_diphone'

class RobotSound(object):
    def __init__(self):
        self._soundhandle = SoundClient(blocking=True)
        count = 0
        self._initialized = False
        while not rospy.is_shutdown() and not self._soundhandle.actionclient.wait_for_server(rospy.Duration(1.0)):
            rospy.logwarn('Waiting for sound play node...')
            count += 1
            if count > 10:
                rospy.logerr('Could not connect to sound play node!')
                return
        self._initialized = True
        rospy.loginfo('Sound play ready')

    def say(self, text):
        if self._initialized:
            self._soundhandle.say(text, VOICE)

    def play_sound(self, sound_file):
        if self._initialized:
            self._soundhandle.playWaveFromPkg('fetch_api', 'sounds/{}'.format(sound_file))

