#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from gz.transport13 import Node
from glob import glob
from os.path import basename
import importlib
import gz.msgs10
import randomproto
import subprocess
import time
from coverage_process import CoverageInfo, GCOV_DIR, BUILD_DIR
import re

def load_builtin(filename):
    with open(filename) as f:
        l = f.read().splitlines()

    return {re.sub(r"/world/.*?/", r"/world/.../", entry) for entry in l}

builtin_services = load_builtin("builtin_services")
builtin_topics = load_builtin("builtin_topics")



non_builtin_services_all = set()
non_builtin_topics_all = set()
for f in glob("./models/*.sdf"):
    gz_sim = f"gz sim {f} -v 0 -r -s --headless-rendering"

    try:
        process = subprocess.Popen(gz_sim.split(" "), stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
    except:
        print("DEBUG: subprocess launch gz error")

    time.sleep(1)

    
    node = Node()
    service_set = {re.sub(r"/world/.*?/", r"/world/.../", entry) for entry in node.service_list()}
    non_builtin_services = service_set - builtin_services
    # print(f"DEBUG: {service_set}")

    topic_set = {re.sub(r"/world/.*?/", r"/world/.../", entry) for entry in node.topic_list()}
    non_builtin_topics = topic_set - builtin_topics

    print(f)
    print(non_builtin_services)
    print(non_builtin_topics)
    non_builtin_services_all |= non_builtin_services
    non_builtin_topics_all |= non_builtin_topics

    subprocess.run("pkill -9 ruby", shell=True)

print(non_builtin_services_all)
print(non_builtin_topics_all)

with open("non_builtin_services", "w") as f:
    for entry in sorted(list(non_builtin_services_all)):
        f.write(f"{entry}\n")

with open("non_builtin_topics", "w") as f:
    for entry in sorted(list(non_builtin_topics_all)):
        f.write(f"{entry}\n")


# c = CoverageInfo(BUILD_DIR, GCOV_DIR)
# c.collect()

###### cmd = "gz sim a.sdf -r 2>&1 | tee gz.replay &"
###### try: 
######     runResult = subprocess.run(["bash", "-c", cmd])
######     for i in range(100):
######         time.sleep(2)
######         print("hello")
###### except Exception as ex:
######     print( f"Failed to run '{cmd}'" )

## DIR = '/data/play/robot/workspace/install/lib/python/gz/msgs10'
## # node = Node()
## # service_list = node.service_list()
## # service = service_list[0]
## # publishers = node.service_info(service)
## # info = publishers[0]
## 
## 
## 
## msg_type_name = 'WorldStatistics'
## 
## 
## class MessageTypeConvert:
##     def __init__(self, directory=DIR):
##         self.file_prefix_list = [basename(i)[:-3] for i in glob(f"{DIR}/*.py")]
##         self.pb2_modules = list()
##         for f in self.file_prefix_list:
##             try:
##                 self.pb2_modules.append(importlib.import_module(f"gz.msgs10.{f}"))
##             except:
##                 print(f"error processing gz.msgs10.{f}")
## 
##     def get_class_type(self, type_name):
##         if type_name.startswith("gz.msgs."):
##             class_type = None
##             for module in self.pb2_modules:
##                 try:
##                     class_type = getattr(module, msg_type_name)
##                     break
##                 except:
##                     continue
## 
##             return class_type
##         else:
##             return None
## 
## 
## 
## 
## process = subprocess.Popen(["gz", "sim", "a.sdf", "-r"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, close_fds=True)
## 
## while True:
##     print("hello")
##     stdout, stderr = process.communicate()

