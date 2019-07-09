#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped, Twist


class CommandHandle(object):
    """Manages timeouts and relaying incoming messages for a given control input"""

    def __init__(self, sub_topic, pub_topic, timeout, stamped):
        self.sub_topic = sub_topic
        self.pub_topic = pub_topic
        self.timeout = float(timeout)
        self.is_stamped = stamped
        self._pub_type = TwistStamped if stamped else Twist

        try:
            self.timeout = float(timeout)
        except ValueError:
            raise ValueError('Invalid `timeout` value for control_input: {TIMEOUT}'
                             .format(TIMEOUT=timeout))

        if not isinstance(stamped, bool):
            raise ValueError('Invalid `stamped` value for control_input: {STAMPED}'
                             .format(STAMPED=stamped))

        self.pub = rospy.Publisher(self.pub_topic, self._pub_type, queue_size=1)
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
        now = rospy.Time.now()
        return 0.0 < self.timeout <= (now - data.header.stamp).to_sec()


class ControlInputManager:
    """Creates a handle for each input.  Drop inputs if timeout is not a number or
    stamped is not a boolean.
    """

    def __init__(self, control_inputs):
        self.input_handles = []
        for idx, control_input in enumerate(control_inputs):
            try:
                command_handle = CommandHandle(**control_input)
                self.input_handles.append(command_handle)
            except ValueError as e:
                rospy.logerr('control_input manager ({IDX}) was not created. {params}'
                             .format(IDX=idx, params=control_input))
                rospy.logerr(e)

    def run(self):
        rospy.spin()


def check_params(control_inputs):
    required_parameters = {'pub_topic', 'sub_topic', 'timeout', 'stamped'}
    for control_input in control_inputs:
        params = set(control_input.keys())
        missing_params = required_parameters - params
        if len(missing_params) != 0:
            err_msg = "Missing the parameters: " + ', '.join(list(missing_params))
            raise ValueError(err_msg)


def main():
    rospy.init_node('control_input_manager')
    control_inputs = rospy.get_param('~control_inputs')
    check_params(control_inputs)
    cim = ControlInputManager(control_inputs)
    cim.run()


if __name__ == '__main__':
    main()
