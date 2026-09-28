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


def get_service_description(lines, package, typ, name, target_output):
    service_description = {
            'package': package,
            'name': name,
            'name_sanitized': name.replace('/', '_').lstrip('_'),
            'name_sanitized_camel': ''.join(word.capitalize() for word in name.replace('/','_').split('_')),
            'typ': typ,
            'typ_snake': ros_camel_to_snake(typ),
            'parameters': [],
            'output': {}
            }
    process_parameters = True
    found_output = False
    outputs = []
    for line in lines:
        line = line.strip()
        if line and not line.startswith('#'):
            if line == '---':
                process_parameters = False
            elif process_parameters:
                parameter_typ, parameter_name = line.split(' ')[:2]
                service_description["parameters"].append({ 'name': parameter_name, 'typ': parameter_typ })
            elif not process_parameters:
                output_typ, output_name = line.split(' ')[:2]
                outputs.append(output_name)
                if output_name == target_output:
                    found_output = True
                    #print("Service {} of type {} used output {}".format(name, typ, output_name))
                    service_description["output"] = { 'name': output_name, 'typ': output_typ }
            else:
                print("Service {} Error: Unexpected service description format:\n{}".format(name, line))
    if not found_output:
        print("Error: Did not find output {} for service {} of type {}.".format(target_output, name, typ))
        print("Possible outputs: {}".format(outputs))
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
            typ, name = line.split(' ')[:2]
            message_description['fields'].append({ 'name': name, 'typ': typ })
    return message_description

services = [
        ('coresense_msgs', 'Understanding.srv')
        ]
messages = [
        ('coresense_msgs', 'Property.msg')
        ]

global_files = ['package.xml.jinja', 'CMakeLists.txt.jinja']

per_service_templates = ['config/call_service.json.jinja', 'behavior_trees/call_service.xml.jinja', 'include/coresense_understanding_interfaces/call_service.hpp.jinja', 'src/call_service.cpp.jinja']

per_modelet_extract_templates = ['config/get_modelet.json.jinja', 'behavior_trees/get_modelet.xml.jinja', 'include/coresense_understanding_interfaces/get_modelet.hpp.jinja', 'src/get_modelet.cpp.jinja']

per_modelet_insert_templates = ['templates/insert_modelet.sparql.tmpl.jinja']



def render(template_file, source, target):
    template = env.get_template(template_file)
    with open (target, 'w') as file:
        #render template
        escaped = template.render(source)
        # unescape jinja syntax
        file.write(escaped.replace('@',''))


if __name__ == '__main__':
    todo = None
    services = []
    modelets = []
    packages = []
    with open('interfaces_config.yaml') as interface_config_file:
        todo = yaml.safe_load(interface_config_file)
    # collect service descriptions
    for service in todo['service_wrappers']:
        package, typ = service["typ"].split('/')
        packages.append(package)
        share_dir = get_package_share_directory(package)
        file_path = join(share_dir, 'srv', typ + '.srv')
        with open(file_path) as service_file:
            services.append(get_service_description(service_file, package, typ, service["name"], service["output"]["field"]))
    # collect modelet descriptions
    for message in todo['triplestar_modelet_retrieval']:
        package, typ = message["typ"].split('/')
        packages.append(package)
        share_dir = get_package_share_directory(package)
        file_path = join(share_dir, 'msg', typ + '.msg')
        with open(file_path) as message_file:
            modelets.append(message | get_message_description(message_file, package, typ))
    # write common files
    for file in global_files:
        template = env.get_template(file)
        with open(file.rpartition('.jinja')[0], 'w') as f:
            f.write(template.render({'modelets': modelets, 'services': services, 'packages': list(set(packages))}))
    # write modelet files
    for modelet in modelets:
        for template in per_modelet_insert_templates:
            path, template_name = split(template)
            file_name = splitext(template_name)[0][7:]
            target_file = join(path, 'insert_' + modelet['package'] + '_' + modelet['typ_snake'] + '_' + file_name)
            render(template, modelet, target_file)
        for template in per_modelet_extract_templates:
            path, template_name = split(template)
            file_name = splitext(template_name)[0][4:]
            target_file = join(path, 'get_' + modelet['package']+ '_' + modelet['typ_snake'] + '_' + file_name)
            render(template, modelet, target_file)

    # write service files
    for service in services:
        for template in per_service_templates:
            path, template_name = split(template)
            file_name = splitext(template_name)[0][5:]
            target_file = join(path, 'call_' + service['name_sanitized'] + '_' + file_name)
            render(template, service, target_file)
