#! /usr/bin/env python
from os.path import splitext, split, join
import re
import sys
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



ros_type_map = {
        "bool": "bool",
        "byte": "uint_8",
        "char": "char",
        "float32": "float",
        "float64": "float",
        "int8": "int8_t",
        "uint8": "uint8_t",
        "int16": "int16_t",
        "uint16": "uin16_t",
        "int32": "int32_t",
        "uint32": "uint32_t",
        "int64": "int64_t",
        "uint64": "uint64_t",
        "string": "std::string"
        }

def ros_camel_to_snake(text):
    text = stage1_input_pattern.sub(output_format, text)
    text = stage2_input_pattern.sub(output_format, text)
    return text.lower() 


def get_service_description(service):
    package, typ = service["typ"].split('/')
    # process concepts, representation_classes, requirements
    packages.append(package)
    share_dir = get_package_share_directory(package)
    file_path = join(share_dir, 'srv', typ + '.srv')
    name = service["name"]
    service_description = {
        'package': package,
        'name': name,
        'name_sanitized': name.replace('/', '_').lstrip('_'),
        'name_sanitized_camel': ''.join(word.capitalize() for word in name.replace('/','_').split('_')),
        'typ': typ,
        'typ_snake': ros_camel_to_snake(typ),
        'parameters': [],
        'output': {
            'concepts': [],
            'representation_classes': [],
            'properties': []
            }
        }
    # concepts and so on belong to parameters and the output
    # output should be simple, just add it to the output.
    target_output = service["output"]["field"]
    with open(file_path) as service_file:
        process_parameters = True
        found_output = False
        outputs = []
        for line in service_file:
            line = line.strip()
            if line and not line.startswith('#'):
                if line == '---':
                    process_parameters = False
                elif process_parameters:
                    ros_typ, parameter_name = line.split(' ')[:2]
                    try:
                        parameter_package, ros_typ = ros_typ.split('/')
                        c_typ = None
                    except ValueError:
                        #parameter_typ = ros_type_map[ros_typ]
                        c_typ = ros_type_map[ros_typ]
                        parameter_package = None
                    parameter = service["inputs"][parameter_name]
                    parameter["name"] = parameter_name
                    parameter["package"] = parameter_package
                    parameter["c_typ"] = c_typ
                    parameter["ros_typ"] = ros_typ
                    service_description["parameters"].append(parameter)
                elif not process_parameters:
                    ros_typ, output_name = line.split(' ')[:2]
                    try:
                        output_package, ros_typ = ros_typ.split('/')
                        c_typ = None
                    except ValueError:
                        c_typ = ros_type_map[ros_typ]
                        #output_typ = ros_typ
                        output_package = None
                    outputs.append(output_name)
                    if output_name == target_output:
                        found_output = True
                        #print("Service {} of ros type {} used output {}".format(name, ros_typ, output_name))
                        service_description["output"]["name"] = output_name
                        service_description["output"]["package"] = output_package
                        service_description["output"]["c_typ"] = c_typ
                        service_description["output"]["ros_typ"] = ros_typ
                        for key in ["concepts", "representation_classes"]:
                            try:
                                service_description["output"][key] = service["output"][key]
                            except KeyError:
                                service_description["output"][key] = []

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
    config_file_path = 'interfaces_config.yaml'
    services = []
    modelets = []
    packages = []
    try:
        with open(config_file_path) as interface_config_file:
            todo = yaml.safe_load(interface_config_file)
        # collect service descriptions
    except FileNotFoundError:
        print("Could not find {}".format(config_file_path))
        sys.exit(1)
    except PermissionError:
        print("Could not read {}".format(config_file_path))
        sys.exit(2)
    except yaml.scanner.ScannerError as e:
        print("Could not parse {}".format(config_file_path))
        print(e)
        sys.exit(3)
    try:
        for service in todo['service_wrappers']:
            services.append(get_service_description(service))
            packages.append(services[-1]["package"])
    except TypeError:
        print("Found no services configured in {}".format(config_file_path))
    # collect modelet descriptions
    try:
        for message in todo['triplestar_modelet_retrieval']:
            package, typ = message["typ"].split('/')
            packages.append(package)
            share_dir = get_package_share_directory(package)
            file_path = join(share_dir, 'msg', typ + '.msg')
            with open(file_path) as message_file:
                modelets.append(message | get_message_description(message_file, package, typ))
    except TypeError:
        print("Found no modelet retrievals configured in {}".format(config_file_path))
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
