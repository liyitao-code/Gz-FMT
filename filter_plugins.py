import re
from collections import OrderedDict

def extract_plugin_blocks(file_path):
    with open(file_path, 'r') as f:
        content = f.read()
    
    # 使用正则表达式匹配完整的plugin块
    plugin_pattern = r'<plugin.*?</plugin>'
    plugins = re.findall(plugin_pattern, content, re.DOTALL)
    
    # 用来存储唯一的plugins，key是filename
    unique_plugins = OrderedDict()
    
    for plugin in plugins:
        # 提取filename
        filename_match = re.search(r'filename="([^"]+)"', plugin)
        if filename_match:
            filename = filename_match.group(1)
            # 只保留每个filename的第一个出现的plugin
            if filename not in unique_plugins:
                unique_plugins[filename] = plugin
    
    # 将结果写入新文件
    output_path = 'unique_plugins.txt'
    with open(output_path, 'w') as f:
        for plugin in unique_plugins.values():
            f.write(plugin + '\n\n')
    
    print(f'Found {len(plugins)} total plugins')
    print(f'Found {len(unique_plugins)} unique plugins')
    print(f'Results saved to {output_path}')

if __name__ == '__main__':
    extract_plugin_blocks('extracted_plugins.txt')
