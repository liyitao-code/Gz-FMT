import xml.etree.ElementTree as ET
import os

sdf_directory = '/home/liyitao/workspace/install/share/gz/gz-sim8/worlds/'
output_file = 'world_level_plugins.txt'

def extract_world_level_plugins(sdf_file):
    """Extracts plugins whose parent is <world>."""
    try:
        tree = ET.parse(sdf_file)
        root = tree.getroot()

        plugins = []
        for world in root.findall('.//world'):
            for plugin in world.findall('plugin'):
                plugins.append(ET.tostring(plugin, encoding='unicode'))
        
        return plugins
    except ET.ParseError as e:
        print(f"Error parsing {sdf_file}: {e}")
        return []

def save_entities_to_file(entities, output_file = output_file):
    """Saves unique entities to a file."""
    with open(output_file, 'w') as f:
        for entity in entities:
            f.write(entity.strip() + "\n\n")  # Ensure consistent format

def retrieve_plugin_in_world_by_index(index, output_file = output_file):
    """Retrieves an entity based on its index from the output file."""
    with open(output_file, 'r') as f:
        content = f.read().strip().split("\n\n")  # Split by two newlines
        if 0 <= index < len(content):
            return content[index]
        else:
            return None

def main_world_plugins():
    # Directory containing SDF files
    

    all_plugins = set()  # Use a set to store unique plugins

    for sdf_filename in os.listdir(sdf_directory):
        sdf_file = os.path.join(sdf_directory, sdf_filename)
        if os.path.isfile(sdf_file) and sdf_file.endswith('.sdf'):
            plugins = extract_world_level_plugins(sdf_file)
            all_plugins.update(plugins)  # Add plugins to the set for uniqueness

    unique_plugins = list(all_plugins)  # Convert set back to list to maintain order
    save_entities_to_file(unique_plugins, output_file)

    # Example: retrieve a plugin by its index
    plugin_index = 0  # Change this to the index you want to retrieve
    plugin_content = retrieve_plugin_in_world_by_index(plugin_index, output_file)
    if plugin_content:
        print(f"Plugin {plugin_index} content:\n{plugin_content}")
    else:
        print(f"Plugin with index {plugin_index} not found.")

if __name__ == '__main__':
    main_world_plugins()
