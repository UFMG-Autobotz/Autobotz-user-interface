#!/usr/bin/env python
import rospy

import sys
from importlib import import_module

class Listener(object):
    def __init__(self, input_var):
        input_list = input_var.split('.')
        self.topic = input_list[0]
        self.attributes = input_list[1:]

        self._binary_sub = rospy.Subscriber(
            self.topic, rospy.AnyMsg, self.binary_callback)

    def binary_callback(self, data):
        assert sys.version_info >= (2,7) #import_module's syntax needs 2.7
        connection_header =  data._connection_header['type'].split('/')

        ros_pkg = connection_header[0] + '.msg'
        msg_type = connection_header[1]
        print 'Message type detected as ' + msg_type

        msg_class = getattr(import_module(ros_pkg), msg_type)

        self._binary_sub.unregister()
        self._deserialized_sub = rospy.Subscriber(
            self.topic, msg_class, self.deserialized_callback)

    def deserialized_callback(self, data):
        for i in self.attributes:
            try:
                data = getattr(data, i)
            except AttributeError:
                raise

        print data



if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    l = Listener('/ball/pose.position.x')
    rospy.spin()


#
#
# class Listener(object):
#     def __init__(self):
#         self._binary_sub = rospy.Subscriber(
#             'some_topic', rospy.AnyMsg, self.binary_callback)
#
#
#
#     def deserialized_callback(self, data):
#         print data.known_field
