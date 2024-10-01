#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import datetime
import psutil
import time

def check(filename):
    if not os.path.exists(filename):
        return
    stat = os.stat(filename)
    ctime = stat.st_mtime
    now = datetime.datetime.now().timestamp()
    with open(filename) as f:
        pid = int(f.read())
    print(pid)

    if now - ctime > 100:
        # kill the pid
        try:
            ps = psutil.Process(pid)
            for child in ps.children(recursive=True):
                child.kill()
            ps.kill()
            with open("logg", "a") as f:
                f.write(f"killed process {pid}\n")
        except Exception as e:
            print(f"Failed to kill pid {pid}: {e}")

if __name__ == "__main__":
    while True:
        check("./terminate")
        # Wait for a specified interval before checking again
        time.sleep(60)  # 每60秒检查一次
