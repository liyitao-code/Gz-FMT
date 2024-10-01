#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from gz.math7 import Vector3d
from gz.sim8 import Model, Link
import random

class TestDriver(object):
    def __init__(self):
        self.id = random.randint(1, 100)
        print("hello")

    def configure(self, entity, sdf, ecm, event_mgr):
        print(entity)
        self.model = Model(entity)
        print(self.model.name(ecm))
        self.link_ids = self.model.links(ecm)
        self.links = [Link(i) for i in self.link_ids]
        self.force = sdf.get_double("force")

    def pre_update(self, info, ecm):
        if info.paused:
            return

        if info.iterations % 3000 == 0:
            for link in self.links:
                link.add_world_force(
                    ecm, Vector3d(0, 0, self.force),
                    Vector3d(random.random(), random.random(), 0))
                print(link.world_pose(ecm))


def get_system():
    return TestDriver()
