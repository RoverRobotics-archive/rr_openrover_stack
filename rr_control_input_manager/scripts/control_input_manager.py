#!/usr/bin/env python
from __future__ import print_function

import rospy
from geometry_msgs.msg import TwistStamped, Twist


class CommandHandle(object):
    def __init__(self, sub_topic, pub_topic, timeout, is_stamped=False):
        self.sub_topic = sub_topic
        self.pub_topic = pub_topic
        self.timeout = timeout
        self.is_stamped = is_stamped

        rospy.loginfo(
            'Configuring Command Input Handler\n' +
            'Subscribed Topic: {TOPIC}\n'.format(TOPIC=self.sub_topic) +
            'Published Topic: {TOPIC}\n'.format(TOPIC=self.pub_topic) +
            'Timeout: {TIMEOUT}\n'.format(TIMEOUT=self.timeout) +
            'Output Topic Stamped: {STAMPED}\n'.format(STAMPED=self.is_stamped)
        )

        self._pub_type = TwistStamped if is_stamped else Twist

        self.sub = rospy.Subscriber(self.sub_topic, TwistStamped, self.callback)
        self.pub = rospy.Publisher(self.pub_topic, self._pub_type, queue_size=10)

    def callback(self, data):
        if not self.expired(data):
            if self.is_stamped:
                self.pub.publish(data)
            else:
                self.pub.publish(data.twist)
        return

    def expired(self, data):
        time_stamp = rospy.Time.now()
        return 0.0 < self.timeout <= (time_stamp - data.header.stamp).to_sec()


class ControlInputManager:
    def __init__(self, command_inputs):
        input_handles = []
        for input in command_inputs:
            pub_topic = input['pub_topic']
            sub_topic = input['sub_topic']
            timeout = input['timeout']
            stamped = input['stamped']

            input_handles.append(CommandHandle(sub_topic, pub_topic, timeout, stamped))

    def start(self):
        rospy.spin()


def check_params(command_inputs):
    print(command_inputs)
    required_parameters = set(['pub_topic', 'sub_topic', 'timeout', 'stamped'])
    for input in command_inputs:
        params = set(input.keys())
        missing_params = required_parameters - params
        if len(missing_params) != 0:
            err_msg = "Missing the parameters: " + ', '.join(list(missing_params))
            raise ValueError(err_msg)


def main():
    rospy.init_node('control_input_manager')
    command_inputs = rospy.get_param('~command_inputs')
    cim = ControlInputManager(command_inputs)
    cim.start()


if __name__ == '__main__':
    print('test')
    main()
