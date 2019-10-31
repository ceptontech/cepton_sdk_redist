#!/usr/bin/env python

from __future__ import (absolute_import, division, generators, nested_scopes,
                        print_function, unicode_literals, with_statement)

import json
import os.path

import numpy

import rospy
import tf


class TransformsNode(object):
    """Publishes transforms from cepton_transforms.json file."""

    def __init__(self):
        rospy.init_node("cepton_transforms")
        self.rate = rospy.Rate(rospy.get_param("~rate", 10))
        self.transform_broadcaster = tf.TransformBroadcaster()

        self.parent_frame_id = rospy.get_param("~parent_frame_id", "cepton")

        self.transforms_dict = {}
        transforms_path = rospy.get_param("~transforms_path", "")
        if not os.path.exists(transforms_path):
            rospy.logwarn("Transforms path does not exist: {}".format(
                          transforms_path))
            return
        with open(transforms_path, "r") as f:
            self.transforms_dict = {
                int(key): value for key, value in json.load(f).items()}

    def publish_transforms(self):
        for serial_number, transform_dict in self.transforms_dict.items():
            translation = transform_dict.get("translation", [0.0, 0.0, 0.0])
            rotation = transform_dict.get("rotation", [0.0, 0.0, 0.0, 1.0])
            frame_id = "cepton_{}".format(serial_number)
            self.transform_broadcaster.sendTransform(
                translation, rotation, rospy.Time.now() + rospy.Duration(1),
                frame_id, self.parent_frame_id)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_transforms()
            self.rate.sleep()


if __name__ == "__main__":
    try:
        node = TransformsNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
