#!/usr/bin/env python

##############################################################################
# Description
##############################################################################

"""
Catches the output from buttons on a joystick and converts these
to customisable ros publishers (Empty, Time Timestamped Bool) for use
as pure events, timestamped events, or true/false triggers.

Publishing only occurs when joystick button signals change state.
"""

##############################################################################
# Imports
##############################################################################

import rospy
from yocs_msgs.msg import MagicButton
from std_msgs.msg import Empty
from std_msgs.msg import Time
from sensor_msgs.msg import Joy

##############################################################################
# Imports
##############################################################################


class EventRelay(object):
    def __init__(self, id, name):
        self.button_id = id 
        self.publisher = rospy.Publisher(name, Empty, queue_size=10)
        rospy.loginfo("MagicButtonRelay : adding event relay for button '%s' to publisher at '%s'" % (self.button_id, self.publisher.name))
        self.last_state = False

    def update(self, pressed, unused_timestamp):
        if not pressed and self.last_state:  # only when released
            self.publisher.publish(Empty())
            rospy.logdebug("MagicButtonRelay : event for button '%s' published at '%s'" % (self.button_id, self.publisher.name))
        self.last_state = pressed

class StampedEventRelay(object):
    def __init__(self, id, name):
        self.button_id = id 
        self.publisher = rospy.Publisher(name, Time, queue_size=10)
        rospy.loginfo("MagicButtonRelay : adding stamped event relay for button '%s' to publisher at '%s'" % (self.button_id, self.publisher.name))
        self.last_state = False

    def update(self, pressed, timestamp):
        if not pressed and self.last_state:  # only when released
            rospy.logdebug("MagicButtonRelay : stamped event for button '%s' published at '%s'" % (self.button_id, self.publisher.name))
            self.publisher.publish(Time(timestamp))
        self.last_state = pressed

class StampedButtonRelay(object):
    def __init__(self, id, name):
        self.button_id = id 
        self.publisher = rospy.Publisher(name, MagicButton, queue_size=10)
        rospy.loginfo("MagicButtonRelay : adding stamped button relay for button '%s' to publisher at '%s'" % (self.button_id, self.publisher.name))
        self.last_state = False

    def update(self, pressed, timestamp):
        if pressed != self.last_state:  # whenever state changes
            msg = MagicButton()
            msg.header.stamp = timestamp
            msg.pressed = pressed
            rospy.logdebug("MagicButtonRelay : stamped button event for button '%s' published at '%s'" % (self.button_id, self.publisher.name))
            self.publisher.publish(msg)
        self.last_state = pressed

class MagicButtonRelay(object):

    def __init__(self, relay_specifications):
        self.relays = []
        self.subscriber = rospy.Subscriber("joy", Joy, self.joy_callback)
        for spec in relay_specifications:
            try:
                if spec["type"] == "event":
                    self.relays.append(EventRelay(spec["id"], spec["name"]))
                elif spec["type"] == "stamped_event":
                    self.relays.append(StampedEventRelay(spec["id"], spec["name"]))
                elif spec["type"] == "stamped_button":
                    self.relays.append(StampedButtonRelay(spec["id"], spec["name"]))
                else:
                    rospy.logerr("MagicButtonRelay : invalid type (%s) for the magic button ['event', 'timestamp', 'button']" % spec["type"])
            except KeyError:
                rospy.logerr("MagicButtonRelay : no 'type' provided for the magic button ['event', 'timestamp', 'button']")
        
    def joy_callback(self, msg):
        """
        Processes the joy topic.
        """
        timestamp = msg.header.stamp
        for relay in self.relays:
            relay.update(msg.buttons[relay.button_id], msg.header.stamp)

if __name__ == '__main__':
    rospy.init_node('magic_button_relay')
    relay_specifications = rospy.get_param("~relays")
    magic_button_relay = MagicButtonRelay(relay_specifications)
    rospy.spin()
