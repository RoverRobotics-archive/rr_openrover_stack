#!/usr/bin/env python
from __future__ import print_function

import rospy
from geometry_msgs.msg import TwistStamped, Twist


class CommandHandle(object):
    """Manages timeouts and relaying incoming messages for a given control input"""

    def __init__(self, sub_topic, pub_topic, timeout, is_stamped=False):
        self.sub_topic = sub_topic
        self.pub_topic = pub_topic
        self.timeout = timeout
        self.is_stamped = is_stamped
        self._pub_type = TwistStamped if is_stamped else Twist
        self.pub = rospy.Publisher(self.pub_topic, self._pub_type, queue_size=5)
        self.sub = rospy.Subscriber(self.sub_topic, TwistStamped, self.callback, queue_size=5)

        rospy.loginfo(
            'Configuring Command Input Handler\n' +
            'Subscribed Topic: {TOPIC}\n'.format(TOPIC=self.sub_topic) +
            'Published Topic: {TOPIC}\n'.format(TOPIC=self.pub_topic) +
            'Timeout: {TIMEOUT}\n'.format(TIMEOUT=self.timeout) +
            'Output Topic Stamped: {STAMPED}\n'.format(STAMPED=self.is_stamped)
        )

    def callback(self, data):
        """Passes through message and strip strips header if necessary"""
        if not self.expired(data):
            if self.is_stamped:
                self.pub.publish(data)
            else:
                self.pub.publish(data.twist)

    def expired(self, data):
        """Tests message stamp to see if it has timeout"""
        time_stamp = rospy.Time.now()
        return 0.0 < self.timeout <= (time_stamp - data.header.stamp).to_sec()


class ControlInputManager:
    """Creates and manages handles for control inputs"""

    def __init__(self, control_inputs):
        self.input_handles = []
        for idx, input in enumerate(control_inputs):
            pub_topic = input['pub_topic']
            sub_topic = input['sub_topic']
            try:
                timeout = float(input['timeout'])
            except ValueError:
                rospy.logerr(
                    'Invalid `timeout` value for control_input {IDX}: {TIMEOUT}'.format(IDX=idx,
                                                                                        TIMEOUT=
                                                                                        input[
                                                                                            'timeout']))
                rospy.logerr('control_input manager ({IDX}) was not created.'.format(IDX=idx))
                continue
            stamped = input['stamped']
            rospy.logwarn(stamped)
            if not isinstance(stamped, bool):
                rospy.logerr(
                    'Invalid `stamped` value for control_input {IDX}: {STAMPED}'.format(IDX=idx,
                                                                                        STAMPED=stamped))
                rospy.logerr('control_input manager ({IDX}) was not created.'.format(IDX=idx))
                continue

            self.input_handles.append(CommandHandle(sub_topic, pub_topic, timeout, stamped))

    def start(self):
        rospy.spin()


def check_params(control_inputs):
    print(control_inputs)
    required_parameters = {'pub_topic', 'sub_topic', 'timeout', 'stamped'}
    for idx, control_input in enumerate(control_inputs):
        params = set(control_input.keys())
        missing_params = required_parameters - params
        if len(missing_params) != 0:
            err_msg = "Missing the parameters: " + ', '.join(list(missing_params))
            raise ValueError(err_msg)


def main():
    rospy.init_node('control_input_manager')
    control_inputs = rospy.get_param('~control_inputs')
    cim = ControlInputManager(control_inputs)
    cim.start()


if __name__ == '__main__':
    print('test')
    main()
