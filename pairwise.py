#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
# TODO: test topic and service combinations

def load_builtin(filename):
    with open(filename) as f:
        l = f.read().splitlines()

    return {re.sub(r"/world/.*?/", r"/world/.../", entry) for entry in l}


builtin_services = load_builtin("builtin_services")
builtin_topics = load_builtin("builtin_topics")


