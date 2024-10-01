#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
import time
import psutil
import os
from glob import glob

def launch_gazebo(sdfname):
    if not os.path.exists(sdfname):
        return
    process = subprocess.Popen(["gz", "sim", sdfname, "-r"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    ps = psutil.Process(process.pid)
    time.sleep(5)

    # process.terminate()
    # process.wait()

    # process.kill()

    for child in ps.children(recursive=True):
        print(child.pid)
        child.terminate()
        child.wait()

    process.terminate()
    print(process.wait())
    print(process.stdout.read().decode("utf-8"))
    print(process.stderr.read().decode("utf-8"))


if __name__ == "__main__":
    launch_gazebo("/home/liyitao/workspace/install/share/gz/gz-sim8/worlds/acoustic_comms.sdf")
