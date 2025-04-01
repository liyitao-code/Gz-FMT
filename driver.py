#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from gz.math8 import Vector3d
from gz.sim9 import Model, Link, Joint
import random

class TestDriver(object):
    def __init__(self):
        self.id = random.randint(1, 100)
        print("hello")

    def configure(self, entity, sdf, ecm, event_mgr):
        print(entity)
        self.model = Model(entity)
        print(self.model.name(ecm))
        self.sdf = sdf

    def pre_update(self, info, ecm):
        self.link_ids = self.model.links(ecm)
        self.joint_ids = self.model.joints(ecm)
        self.joints = [Joint(i) for i in self.joint_ids]
        self.links = [Link(i) for i in self.link_ids]
        self.force = self.sdf.get_double("force")
        if info.paused:
            return

        if info.iterations % 1000 == 0:
            for link in self.links:
                x_force = (2 * random.random() - 1) * self.force
                y_force = (2 * random.random() - 1) * self.force
                z_force = (2 * random.random() - 1) * self.force

                link.add_world_wrench(
                    ecm, Vector3d(x_force, y_force, z_force),
                    Vector3d(random.random() * self.force, random.random() * self.force, random.random() * self.force))
                    # Vector3d(random.random(), random.random(), 0))
                print(link.world_pose(ecm))
            for joint in self.joints:
                joint.set_velocity(
                    ecm, [random.random() * self.force, random.random() * self.force, random.random() * self.force])
                    # Vector3d(random.random(), random.random(), 0))


def get_system():
    return TestDriver()
