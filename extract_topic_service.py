#!/usr/bin/python3
# coding: utf-8

from glob import glob
import re

def load_builtin(filename):
    with open(filename) as f:
        l = f.read().splitlines()

    return {re.sub(r"/world/.*?/", r"/world/.../", entry) for entry in l}
builtin_services = load_builtin("builtin_services")
builtin_topics = load_builtin("builtin_topics")

services = set()
topics = set()
for f in glob("**/cmd_*.sh"):
    with open(f) as o:
        content = o.read()

    m = re.match(r"gz ((service .* -s)|(topic -t)) (.*?) .*", content)
    if m:
        print(m.groups())
        if m.group(1).startswith("service"):
            service = re.sub(r"/world/.*?/", "/world/.../", m.group(4))
            services.add(service)
        if m.group(1).startswith("topic"):
            topic = re.sub(r"/world/.*?/", "/world/.../", m.group(4))
            topics.add(topic)

print(services - builtin_services)
print(topics - builtin_topics)

with open("executed_services", "w") as f:
    for entry in sorted(list(services - builtin_services)):
        f.write(f"{entry}\n")

with open("executed_topics", "w") as f:
    for entry in sorted(list(topics - builtin_topics)):
        f.write(f"{entry}\n")
