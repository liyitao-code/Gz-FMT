#!/usr/bin/env python3

import sys
sys.path.append('/home/liyitao/workspace/gazebo/install/lib/python')
sys.path.append('/home/liyitao/workspace/gazebo/src/gz-sim/python/test/')

import os
import unittest

from gz_test_deps.common import set_verbosity
from gz_test_deps.sim import K_NULL_ENTITY, TestFixture, Model, World, world_entity

class TestGazebo(unittest.TestCase):
    def setUp(self):
        # This will be called before each test function
        set_verbosity(4)

    def test_load_sdf_and_model(self):
        # Ensure there's a command line argument for the SDF file path
        if len(sys.argv) < 2:
            print("Usage: python test_script.py <path_to_sdf_file>")
            sys.exit(1)
        
        sdf_file_path = sys.argv[1]
        if not os.path.isfile(sdf_file_path):
            print(f"Error: The specified file does not exist: {sdf_file_path}")
            sys.exit(1)

        # Create a TestFixture with the given SDF file
        fixture = TestFixture(sdf_file_path)

        # Callback to verify model loading
        # def on_pre_update_cb(_info, _ecm):
        #     world_e = world_entity(_ecm)
        #     self.assertNotEqual(K_NULL_ENTITY, world_e, "World entity not found")
        #     world = World(world_e)
            
        #     # Check for a specific model in the SDF
        #     model_name = 'your_model_name'  # Replace with your actual model name
        #     model = Model(world.model_by_name(_ecm, model_name))
        #     self.assertNotEqual(K_NULL_ENTITY, model.entity(), f"Model '{model_name}' not found")
        #     self.assertTrue(model.valid(_ecm), f"Model '{model_name}' is not valid")

        # Register the callback
        fixture.on_pre_update(on_pre_update_cb)
        fixture.finalize()

        # Run the server for 1 iteration to ensure the model is loaded
        server = fixture.server()
        server.run(True, 1, False)
        print("TEST PASSED")

if __name__ == '__main__':
    # Remove the first argument which is the script name
    unittest.main(argv=sys.argv[0:1])
