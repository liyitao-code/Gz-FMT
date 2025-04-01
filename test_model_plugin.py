#!/usr/bin/env python3

from muti_agent_smith_testfixture import SmithUnit
import os

def test_add_model_with_plugin():
    # Initialize the SmithUnit
    test_dir = "test_exp"
    if not os.path.exists(test_dir):
        os.makedirs(test_dir)
        
    smith = SmithUnit(directory=test_dir)
    
    # Create an initial world file
    smith.create_sdf(dump=True)
    
    # Set up the test environment
    smith.setUp()
    
    # Launch Gazebo
    smith.launch_gazebo()
    
    try:
        # Try to add a model with plugin
        result = smith.func_add_random_model_with_plugin()
        if result is None:
            print("Failed to add model with plugin")
        else:
            print("Successfully created model with plugin command")
            
            # Execute the command
            result.execute()
            print("Successfully executed model with plugin command")
    except Exception as e:
        print(f"Error during test: {str(e)}")
    finally:
        # Clean up
        smith.stop_gazebo()

if __name__ == "__main__":
    test_add_model_with_plugin()
