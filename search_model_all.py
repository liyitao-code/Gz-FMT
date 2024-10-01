import xml.etree.ElementTree as ET
import os

sdf_directory = '/home/liyitao/workspace/rezilla-modelsmith-fb63e64b5fab/models/'
output_file = 'models_all.txt'
def extract_models_with_plugins(sdf_file):
    """Extracts models that contain at least one plugin."""
    try:
        tree = ET.parse(sdf_file)
        root = tree.getroot()

        models = []
        # for world in root.findall('.//world'):
        #     for model in world.findall('model'):
        #         models.append(ET.tostring(model, encoding='unicode'))
        for model in root.findall('.//model'):
            models.append(ET.tostring(model, encoding='unicode'))
        return models
    except ET.ParseError as e:
        print(f"Error parsing {sdf_file}: {e}")
        return []

def save_model_to_file(plugins, output_file = output_file):
    """Saves unique plugins to a file."""
    with open(output_file, 'w') as f:
        for plugin in plugins:
            f.write(plugin.strip() + "\n\n")  # Ensure consistent format

def retrieve_model_by_index(index, output_file = output_file):
    """Retrieves a plugin based on its index from the output file."""
    with open(output_file, 'r') as f:
        content = f.read().strip().split("\n\n")  # Split by two newlines
        if 0 <= index < len(content):
            return content[index]
        else:
            return None

def main_models_with_plugins():
    # Directory containing SDF files

    all_models = set()  # Use a set to store unique models

    for sdf_filename in os.listdir(sdf_directory):
        sdf_file = os.path.join(sdf_directory, sdf_filename)
        if os.path.isfile(sdf_file) and sdf_file.endswith('.sdf'):
            models = extract_models_with_plugins(sdf_file)
            all_models.update(models)  # Add models to the set for uniqueness

    unique_models = list(all_models)  # Convert set back to list to maintain order
    save_model_to_file(unique_models, output_file)
    print(f"number is {len(unique_models)}")

    # Example: retrieve a model by its index
    model_index = 0  # Change this to the index you want to retrieve
    model_content = retrieve_model_by_index(model_index, output_file)
    if model_content:
        print(f"Model {model_index} content:\n{model_content}")
    else:
        print(f"Model with index {model_index} not found.")

if __name__ == '__main__':
    main_models_with_plugins()

