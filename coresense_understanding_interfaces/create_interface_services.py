#! /usr/bin/env python
from os.path import splitext, split, join
import re
import yaml
from jinja2 import Environment, PackageLoader, select_autoescape, FileSystemLoader
from ament_index_python.packages import get_package_share_directory


env = Environment(
    loader=FileSystemLoader("."),
    autoescape=select_autoescape()
)

env.trim_blocks = True
#env.lstrip_blocks = True

output_format = r'\1_\2'
# insert an underscore before any upper case letter
# which is not followed by another upper case letter
stage1_input_pattern = re.compile(r'(.)([A-Z][a-z]+)')

# insert an underscore before any upper case letter
# which is preseded by a lower case letter or number
stage2_input_pattern = re.compile(r'([a-z0-9])([A-Z])')

def ros_camel_to_snake(text):
    text = stage1_input_pattern.sub(output_format, text)
    text = stage2_input_pattern.sub(output_format, text)
    return text.lower() 


def get_service_description(lines, package, typ, name):
    service_description = {
            'package': package,
            'name': name,
            'name_sanitized': name.replace('/', '_').lstrip('_'),
            'name_sanitized_camel': ''.join(word.capitalize() for word in name.replace('/','_').split('_')),
            'typ': typ,
            'typ_snake': ros_camel_to_snake(typ),
            'parameters': [],
            'outputs': []
            }
    phases = ['parameters', 'outputs']
    index = 0
    for line in lines:
        line = line.strip()
        if line and not line.startswith('#'):
            if line == '---':
                index += 1
            else:
                service_description[phases[index]].append(line.split(' ')[:2])
    if len(service_description['outputs']) > 1:
        print("WARNING: Engines can only have one output. Therefore, services with more than one output are not supported.")
    return service_description

def get_message_description(lines, package, typ):
    message_description = {
            'package': package,
            'typ': typ,
            'typ_snake': ros_camel_to_snake(typ),
            'fields': []
            }
    for line in lines:
        line = line.strip()
        if line and not line.startswith('#'):
            message_description['fields'].append(line.split(' ')[:2])
    return message_description

services = [
        ('coresense_msgs', 'Understanding.srv')
        ]
messages = [
        ('coresense_msgs', 'Property.msg')
        ]

global_files = ['package.xml.jinja', 'CMakeLists.txt.jinja']

per_service_files = ['config/call_service.json.jinja', 'behavior_trees/call_service.xml.jinja', 'include/coresense_understanding_interfaces/call_service.hpp.jinja', 'src/call_service.cpp.jinja']

per_modelet_files = ['config/get_modelet.json.jinja', 'behavior_trees/get_modelet.xml.jinja', 'include/coresense_understanding_interfaces/get_modelet.hpp.jinja', 'src/get_modelet.cpp.jinja']

if __name__ == '__main__':
    todo = None
    services = []
    modelets = []
    packages = []
    with open('interfaces_config.yaml') as interface_config_file:
        todo = yaml.safe_load(interface_config_file)
    # collect service descriptions
    for [service_type, name] in todo['service_wrappers']:
        package, typ = service_type.split('/')
        packages.append(package)
        share_dir = get_package_share_directory(package)
        file_path = join(share_dir, 'srv', typ + '.srv')
        with open(file_path) as service_file:
            services.append(get_service_description(service_file, package, typ, name))
    # collect modelet descriptions
    for [message_type] in todo['triplestar_modelet_retrieval']:
        package, typ = message_type.split('/')
        packages.append(package)
        share_dir = get_package_share_directory(package)
        file_path = join(share_dir, 'msg', typ + '.msg')
        with open(file_path) as message_file:
            modelets.append(get_message_description(message_file, package, typ))
    # write common files
    for file in global_files:
        template = env.get_template(file)
        with open(file.rpartition('.jinja')[0], 'w') as f:
            f.write(template.render({'modelets': modelets, 'services': services, 'packages': list(set(packages))}))
    # write modelet files
    for modelet in modelets:
        for file in per_modelet_files:
            template = env.get_template(file)
            path, template_name = split(file)
            file_name = splitext(template_name)[0][4:]
            target_file = join(path, 'get_' + modelet['package']+ '_' + modelet['typ_snake'] + '_' + file_name)

            with open(target_file, 'w') as f:
                f.write(template.render(modelet))

    # write service files
    for service in services:
        for file in per_service_files:
            template = env.get_template(file)
            path, template_name = split(file)
            file_name = splitext(template_name)[0][5:]
            target_file = join(path, 'call_' + service['name_sanitized'] + '_' + file_name)

            with open(target_file, 'w') as f:
                f.write(template.render(service))

