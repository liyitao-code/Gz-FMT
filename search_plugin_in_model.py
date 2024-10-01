import xml.etree.ElementTree as ET
import os

sdf_directory = '/home/liyitao/workspace/install/share/gz/gz-sim8/worlds/'
output_file = 'extracted_plugins.txt'

def extract_plugins_from_sdf(sdf_file):
    """Extracts plugin elements from given SDF file."""
    try:
        tree = ET.parse(sdf_file)
        root = tree.getroot()

        plugins = []
        for model in root.findall('.//model'):
            for plugin in model.findall('plugin'):
                plugins.append(ET.tostring(plugin, encoding='unicode'))
        
        return plugins
    except ET.ParseError as e:
        print(f"Error parsing {sdf_file}: {e}")
        return []

def save_plugins_to_file(plugins, output_file = output_file):
    """Saves unique plugins to a file."""
    with open(output_file, 'w') as f:
        for plugin in plugins:
            f.write(plugin.strip() + "\n\n")  # Ensure consistent format

def retrieve_plugin_by_index(index, output_file = output_file):
    """Retrieves a plugin based on its index from the output file."""
    with open(output_file, 'r') as f:
        content = f.read().strip().split("\n\n")  # Split by two newlines
        if 0 <= index < len(content):
            return content[index]
        else:
            return None

def main():
    # Directory containing SDF files
    

    all_plugins = set()  # Use a set to store unique plugins

    for sdf_filename in os.listdir(sdf_directory):
        sdf_file = os.path.join(sdf_directory, sdf_filename)
        if os.path.isfile(sdf_file) and sdf_file.endswith('.sdf'):
            plugins = extract_plugins_from_sdf(sdf_file)
            all_plugins.update(plugins)  # Add plugins to the set for uniqueness

    unique_plugins = list(all_plugins)  # Convert set back to list to maintain order
    save_plugins_to_file(unique_plugins, output_file)

    # Example: retrieve a plugin by its index
    for plugin_index in range(0, 130):
        print(f"plugin index is {plugin_index}")
        # plugin_index = 120
        plugin_content = retrieve_plugin_by_index(plugin_index, output_file)
        if plugin_content:
            print(f"Plugin {plugin_index} content:\n{plugin_content}")
        else:
            print(f"Plugin with index {plugin_index} not found.")

if __name__ == '__main__':
    main()
