import os

def find_text_in_sdf_files(directory, search_text):
    matching_files = []
    # Walk through the directory
    for root, _, files in os.walk(directory):
        
        for file in files:
            # print(file)
            if file.endswith('.sdf'):
                file_path = os.path.join(root, file)
                
                try:
                    with open(file_path, 'r', encoding='utf-8') as f:
                        
                        contents = f.read()
                        if search_text in contents:
                            matching_files.append(file_path)
                except Exception as e:
                    print(f"Error reading {file_path}: {e}")
    return matching_files

# Input from the user
directory = "/home/liyitao/workspace/rezilla-modelsmith-fb63e64b5fab/models"
# search_text = """<atmosphere type='adiabatic'>"""

search_text = """gz::sim::systems::LinearBatteryPlugin"""

# Find and print matching .sdf files
matching_files = find_text_in_sdf_files(directory, search_text)
if matching_files:
    print("Files containing the text:")
    for file in matching_files:
        print(file)
else:
    print("No .sdf files contain the specified text.")



