#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from service_smith import SmithUnit

class Fuzz:
    def __init__(self, num_units = 100):
        # def __init__(self, directory="exp", sdf_name="a.sdf", num_seq=10, use_text=True, skipped=None, timeout=10000, seed=0):
        self.units = []
        self.num_units = num_units

    def populate(self, directory="exp", sdf_name="a.sdf", num_seq=10, use_text=True, skipped=None, timeout=10000, seed=0, miner=None):
        for i in range(num_units):
            unit = SmithUnit(directory, sdf_name, num_seq, use_text, skipped, timeout, seed, miner)
            self.units.append(unit)

    def crossover(self):
        pass

    def mutate(self):
        pass

    def select(self):
        pass

    def loop(self):
        pass



class FuzzBackup:
    def __init__(self, mode="cmd"):
        self.node = Node()
        self.mode = mode # cmd or obj


    def one_shot_fuzz(self, directory, iteration, cmd_post="cmd", out_post="out"):
        service_list = self.node.service_list()
        topic_list = self.node.topic_list()

        if not os.path.exists(directory):
            os.mkdir(directory)

        # 1. random select an action
        func_name = random.choice(self.funcs)
        print(func_name)
        func = getattr(self, func_name)
        # 2. apply the action
        cmd, result, response = func()
        if type(cmd) == list:
            # topic func, dirty...
            # there's no result/response for topics
            with open(f"{directory}/cmd_{iteration}.{cmd_post}", "w") as out:
                for c in cmd:
                    out.write(c)
        else:
            with open(f"{directory}/cmd_{iteration}.{cmd_post}", "w") as out:
                out.write(cmd)
            with open(f"{directory}/cmd_{iteration}.{out_post}", "w") as out:
                out.write(str(response))

    def fuzz(self, directory=".", cmd_post="cmd", out_post="out"):
        # TODO: collect coverage information at each iteration
        print("DEBUG: before cov_old.collect()")
        self.cov_old.collect()
        service_list = self.node.service_list()
        topic_list = self.node.topic_list()

        if not os.path.exists(directory):
            os.mkdir(directory)
        iteration = 0
        print(self.cov_old.file_cov)
        while service_list and topic_list and iteration < 1:
            # 1. generate current sdf world
            world_content = self.dump_sdf(self.world_name)
            with open(f"{directory}/world_{iteration}.sdf", "w") as out:
                out.write(world_content)
            # 2. random select an action
            func_name = random.choice(self.funcs)
            print(func_name)
            func = getattr(self, func_name)
            # 3. apply the action
            cmd, result, response = func()
            if type(cmd) == list:
                # topic func, dirty...
                # there's no result/response for topics
                with open(f"{directory}/cmd_{iteration}.{cmd_post}", "w") as out:
                    for c in cmd:
                        out.write(c)
            else:
                with open(f"{directory}/cmd_{iteration}.{cmd_post}", "w") as out:
                    out.write(cmd)
                with open(f"{directory}/cmd_{iteration}.{out_post}", "w") as out:
                    out.write(str(response))
            # 4. how to collect the stack trace?
            service_list = self.node.service_list()
            topic_list = self.node.topic_list()
            iteration += 1
            with open(f"{directory}/id", "w") as f:
                f.write(f"{iteration}")

            # collect coverage
            self.cov_new = CoverageInfo(BUILD_DIR, GCOV_DIR)
            self.cov_new.collect()
            diff = CoverageDiff()
            diff.compare(self.cov_new, self.cov_old)

            # write diff to a file?
            self.cov_old = self.cov_new
            with open(f"{directory}/covdiff.txt", "a") as out:
                out.write(f"new_line: {diff.new_line}, new_file: {diff.new_file}\n")

