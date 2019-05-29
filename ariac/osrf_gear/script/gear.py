#!/usr/bin/env python

# Software License Agreement (Apache License)
#
# Copyright 2016 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import print_function

import argparse
import math
import os
import subprocess
import sys

import roslaunch

import em
import yaml

this_dir = os.path.abspath(os.path.dirname(__file__))
template_files = [
    os.path.join(this_dir, '..', '..', 'share', 'osrf_gear', 'worlds', 'gear.world.template'),
    os.path.join(this_dir, '..', '..', 'share', 'osrf_gear', 'launch', 'gear.launch.template'),
    os.path.join(this_dir, '..', '..', 'share', 'osrf_gear', 'launch', 'gear.urdf.xacro.template'),
]
arm_configs = {
    'ur10': {
        'default_initial_joint_states': {
            'linear_arm_actuator_joint': 0,
            'shoulder_pan_joint': 3.14,
            'shoulder_lift_joint': -1.13,
            'elbow_joint': 1.51,
            'wrist_1_joint': 3.77,
            'wrist_2_joint': -1.51,
            'wrist_3_joint': 0,
        }
    },
}
sensor_configs = {
    'break_beam': None,
    'proximity_sensor': None,
    'logical_camera': None,
    'laser_profiler': None,
}
default_bin_origins = {
    'bin1': [-1.0, -1.33, 0],
    'bin2': [-1.0, -0.535, 0],
    'bin3': [-1.0, 0.23, 0],
    'bin4': [-1.0, 0.995, 0],
    'bin5': [-0.3, -1.33, 0],
    'bin6': [-0.3, -0.535, 0],
    'bin7': [-0.3, 0.23, 0],
    'bin8': [-0.3, 0.995, 0],
}
bin_width = 0.6
bin_height = 0.72
configurable_options = {
    'insert_agvs': True,
    'insert_models_over_bins': False,
    'disable_shadows': False,
    'fill_demo_tray': False,
    'belt_population_cycles': 1,
    'gazebo_state_logging': False,
    'spawn_extra_models': False,
    'unthrottled_physics_update': False,
    'model_type_aliases': {
        'belt_model_type1': 'part1',
        'belt_model_type2': 'part2',
    },
}
default_time_limit = 500  # seconds
global_model_count = {}  # the global count of how many times a model type has been created


def prepare_arguments(parser):
    add = parser.add_argument
    add('-n', '--dry-run', action='store_true', default=False,
        help='print generated files to stdout, but do not write them to disk')
    add('-v', '--verbose', action='store_true', default=False,
        help='output additional logging to console')
    add('-o', '--output', default='/tmp/ariac/',
        help='directory in which to output the generated files')
    add('--development-mode', '-d', action='store_true', default=False,
        help='if true the competition mode environment variable will not be set (default false)')
    add('--no-gui', action='store_true', default=False,
        help="don't run the gazebo client gui")
    mex_group = parser.add_mutually_exclusive_group(required=False)
    add = mex_group.add_argument
    add('config', nargs="?", metavar="CONFIG",
        help='yaml string that is the configuration')
    add('-f', '--file', nargs='+', help='list of paths to yaml files that contain the '
        'configuration (contents will be concatenated)')


eval_local_vars = {n: getattr(math, n) for n in dir(math) if not n.startswith('__')}


def expand_to_float(val):
    if isinstance(val, str):
        return float(eval(val, {}, eval_local_vars))
    return float(val)


def expand_yaml_substitutions(yaml_dict):
    for k, v in yaml_dict.items():
        if isinstance(v, dict):
            yaml_dict[k] = expand_yaml_substitutions(v)
        if k in ['xyz', 'rpy']:
            yaml_dict[k] = [expand_to_float(x) for x in v]
        if k in ['initial_joint_states']:
            yaml_dict[k] = {kp: expand_to_float(vp) for kp, vp in v.items()}
    return yaml_dict


class ArmInfo:
    def __init__(self, arm_type, initial_joint_states):
        self.type = arm_type
        self.initial_joint_states = initial_joint_states


class ModelInfo:
    def __init__(self, model_type, pose, reference_frame):
        self.type = model_type
        self.pose = pose
        self.reference_frame = reference_frame


class SensorInfo:
    def __init__(self, name, sensor_type, pose):
        self.name = name
        self.type = sensor_type
        self.pose = pose


class PoseInfo:
    def __init__(self, xyz, rpy):
        self.xyz = [str(f) for f in xyz]
        self.rpy = [str(f) for f in rpy]


class DropRegionInfo:
    def __init__(self, drop_region_min, drop_region_max, destination, model_type):
        self.min = [str(f) for f in drop_region_min]
        self.max = [str(f) for f in drop_region_max]
        self.destination = destination
        self.type = model_type


def get_field_with_default(data_dict, entry, default_value):
    if entry in data_dict:
        return data_dict[entry]
    else:
        return default_value


def get_required_field(entry_name, data_dict, required_entry):
    if required_entry not in data_dict:
        print("Error: '{0}' entry does not contain a required '{1}' entry"
              .format(entry_name, required_entry),
              file=sys.stderr)
        sys.exit(1)
    return data_dict[required_entry]


def replace_type_aliases(model_type):
    if model_type in configurable_options['model_type_aliases']:
        model_type = configurable_options['model_type_aliases'][model_type]
    return model_type


def model_count_post_increment(model_type):
    global global_model_count
    try:
        count = global_model_count[model_type]
    except KeyError:
        count = 1
    global_model_count[model_type] = count + 1
    return count


def create_arm_info(arm_dict):
    arm_type = get_required_field('arm', arm_dict, 'type')
    if arm_type not in arm_configs:
        print("Error: arm type '{0}' is not one of the valid arm entries: {1}"
              .format(arm_type, arm_configs), file=sys.stderr)
        sys.exit(1)
    for key in arm_dict:
        if key != 'type':
            print(
                "Warning: ignoring unknown entry in '{0}': {1}"
                .format('arm', key), file=sys.stderr)
    initial_joint_states = arm_configs[arm_type]['default_initial_joint_states']
    return ArmInfo(arm_type, initial_joint_states)


def create_pose_info(pose_dict):
    xyz = get_field_with_default(pose_dict, 'xyz', [0, 0, 0])
    rpy = get_field_with_default(pose_dict, 'rpy', [0, 0, 0])
    for key in pose_dict:
        if key not in ['xyz', 'rpy']:
            print("Warning: ignoring unknown entry in 'pose': " + key, file=sys.stderr)
    return PoseInfo(xyz, rpy)


def create_sensor_info(name, sensor_data):
    sensor_type = get_required_field(name, sensor_data, 'type')
    pose_dict = get_required_field(name, sensor_data, 'pose')
    for key in sensor_data:
        if key not in ['type', 'pose']:
            print("Warning: ignoring unknown entry in '{0}': {1}"
                  .format(name, key), file=sys.stderr)
    if sensor_type not in sensor_configs:
        print("Error: given sensor type '{0}' is not one of the known sensor types: {1}"
              .format(sensor_type, sensor_configs.keys()), file=sys.stderr)
    pose_info = create_pose_info(pose_dict)
    return SensorInfo(name, sensor_type, pose_info)


def create_sensor_infos(sensors_dict):
    sensor_infos = {}
    for name, sensor_data in sensors_dict.items():
        sensor_infos[name] = create_sensor_info(name, sensor_data)
    return sensor_infos


def create_model_info(model_name, model_data):
    model_type = get_required_field(model_name, model_data, 'type')
    model_type = replace_type_aliases(model_type)
    pose_dict = get_required_field(model_name, model_data, 'pose')
    reference_frame = get_field_with_default(model_data, 'reference_frame', '')
    for key in model_data:
        if key not in ['type', 'pose', 'reference_frame']:
            print("Warning: ignoring unknown entry in '{0}': {1}"
                  .format(model_name, key), file=sys.stderr)
    pose_info = create_pose_info(pose_dict)
    return ModelInfo(model_type, pose_info, reference_frame)


def create_models_to_spawn_infos(models_to_spawn_dict):
    models_to_spawn_infos = {}
    for reference_frame, reference_frame_data in models_to_spawn_dict.items():
        models = get_required_field(reference_frame, reference_frame_data, 'models')
        for model_name, model_to_spawn_data in models.items():
            model_to_spawn_data['reference_frame'] = reference_frame
            model_info = create_model_info(model_name, model_to_spawn_data)
            # assign each model a unique name because gazebo can't do this
            # if the models all spawn at the same time
            scoped_model_name = reference_frame.replace('::', '|') + '|' + \
                model_info.type + '_' + str(model_count_post_increment(model_info.type))
            models_to_spawn_infos[scoped_model_name] = model_info
    return models_to_spawn_infos


def create_models_over_bins_infos(models_over_bins_dict):
    models_to_spawn_infos = {}
    for bin_name, bin_dict in models_over_bins_dict.items():
        if bin_name in default_bin_origins:
            offset_xyz = [
                default_bin_origins[bin_name][0] - bin_width / 2,
                default_bin_origins[bin_name][1] - bin_width / 2,
                bin_height + 0.03]
            # Allow the origin of the bin to be over-written
            if 'xyz' in bin_dict:
                offset_xyz = bin_dict['xyz']
        else:
            offset_xyz = get_required_field(bin_name, bin_dict, 'xyz')

        models = get_required_field(bin_name, bin_dict, 'models') or {}
        for model_type, model_to_spawn_dict in models.items():
            model_to_spawn_data = {}
            model_to_spawn_data['type'] = model_type
            model_to_spawn_data['reference_frame'] = 'world'
            xyz_start = get_required_field(
                model_type, model_to_spawn_dict, 'xyz_start')
            xyz_end = get_required_field(
                model_type, model_to_spawn_dict, 'xyz_end')
            rpy = get_required_field(model_type, model_to_spawn_dict, 'rpy')
            num_models_x = get_required_field(
                model_type, model_to_spawn_dict, 'num_models_x')
            num_models_y = get_required_field(
                model_type, model_to_spawn_dict, 'num_models_y')
            step_size = [
                (xyz_end[0] - xyz_start[0]) / max(1, num_models_x - 1),
                (xyz_end[1] - xyz_start[1]) / max(1, num_models_y - 1)]

            # Create a grid of models
            for idx_x in range(num_models_x):
                for idx_y in range(num_models_y):
                    xyz = [
                        offset_xyz[0] + xyz_start[0] + idx_x * step_size[0],
                        offset_xyz[1] + xyz_start[1] + idx_y * step_size[1],
                        offset_xyz[2] + xyz_start[2]]
                    model_to_spawn_data['pose'] = {'xyz': xyz, 'rpy': rpy}
                    model_info = create_model_info(model_type, model_to_spawn_data)
                    # assign each model a unique name because gazebo can't do this
                    # if the models all spawn at the same time
                    scoped_model_name = bin_name + '|' + \
                        model_info.type + '_' + str(model_count_post_increment(model_type))
                    model_info.bin = bin_name
                    models_to_spawn_infos[scoped_model_name] = model_info
    return models_to_spawn_infos


def create_belt_part_infos(belt_parts_dict):
    belt_part_infos = {}
    for obj_type, spawn_times in belt_parts_dict.items():
        for spawn_time, belt_part_dict in spawn_times.items():
            obj_type = replace_type_aliases(obj_type)
            if obj_type not in belt_part_infos:
                belt_part_infos[obj_type] = {}
            belt_part_dict['type'] = obj_type
            belt_part_infos[obj_type][spawn_time] = create_model_info('belt_part', belt_part_dict)
    return belt_part_infos


def create_drops_info(drops_dict):
    drops_info = {}
    drop_region_infos = []
    drop_regions_dict = get_required_field('drops', drops_dict, 'drop_regions')
    for drop_name, drop_region_dict in drop_regions_dict.items():
        drop_region_min = get_required_field('drop_region', drop_region_dict, 'min')
        drop_region_min_xyz = get_required_field('min', drop_region_min, 'xyz')
        drop_region_max = get_required_field('drop_region', drop_region_dict, 'max')
        drop_region_max_xyz = get_required_field('max', drop_region_max, 'xyz')
        destination_info = get_required_field('drop_region', drop_region_dict, 'destination')
        destination = create_pose_info(destination_info)
        part_type = get_required_field('drop_region', drop_region_dict, 'part_type_to_drop')
        part_type = replace_type_aliases(part_type)
        drop_region_infos.append(
            DropRegionInfo(drop_region_min_xyz, drop_region_max_xyz, destination, part_type))
    drops_info['drop_regions'] = drop_region_infos
    return drops_info


def create_order_info(name, order_dict):
    kit_count = get_field_with_default(order_dict, 'kit_count', 1)
    announcement_condition = get_required_field(name, order_dict, 'announcement_condition')
    announcement_condition_value = get_required_field(name, order_dict, 'announcement_condition_value')
    parts_dict = get_required_field(name, order_dict, 'parts')
    parts = []
    for part_name, part_dict in parts_dict.items():
        parts.append(create_model_info(part_name, part_dict))
    return {
        'announcement_condition': announcement_condition,
        'announcement_condition_value': announcement_condition_value,
        'parts': parts,
        'kit_count': kit_count,
    }


def create_order_infos(orders_dict):
    order_infos = {}
    for order_name, order_dict in orders_dict.items():
        order_infos[order_name] = create_order_info(order_name, order_dict)
    return order_infos


def create_faulty_parts_info(faulty_parts_dict):
    faulty_part_infos = {}
    for part_name in faulty_parts_dict:
        faulty_part_infos[part_name] = part_name  # no other info for now
    return faulty_part_infos


def create_bin_infos():
    bin_infos = {}
    for bin_name, xyz in default_bin_origins.items():
        bin_infos[bin_name] = PoseInfo(xyz, [0, 0, 1.5708])
    return bin_infos


def create_material_location_info(belt_parts, models_over_bins):
    # Specify where trays can be found
    material_locations = {'tray': set(['agv1_load_point', 'agv2_load_point'])}

    # Specify that belt parts can be found on the conveyor belt
    for _, spawn_times in belt_parts.items():
        for spawn_time, part in spawn_times.items():
            if part.type in material_locations:
                material_locations[part.type].update(['belt'])
            else:
                material_locations[part.type] = set(['belt'])

    # Specify in which bin the different bin parts can be found
    for part_name, part in models_over_bins.items():
        if part.type in material_locations:
            material_locations[part.type].update([part.bin])
        else:
            material_locations[part.type] = set([part.bin])

    return material_locations


def create_options_info(options_dict):
    options = configurable_options
    for option, val in options_dict.items():
        options[option] = val
    return options


def prepare_template_data(config_dict):
    template_data = {
        'arm': None,
        'sensors': {},
        'models_to_insert': {},
        'models_to_spawn': {},
        'belt_parts': {},
        'faulty_parts': {},
        'drops': {},
        'orders': {},
        'options': {},
        'time_limit': default_time_limit,
        'bin_height': bin_height,
    }
    # Process the options first as they may affect the processing of the rest
    options_dict = get_field_with_default(config_dict, 'options', {})
    template_data['options'].update(create_options_info(options_dict))

    models_over_bins = {}
    for key, value in config_dict.items():
        if key == 'arm':
            template_data['arm'] = create_arm_info(value)
        elif key == 'sensors':
            template_data['sensors'].update(
                create_sensor_infos(value))
        elif key == 'models_over_bins':
            models_over_bins = create_models_over_bins_infos(value)
            template_data['models_to_insert'].update(models_over_bins)
        elif key == 'belt_parts':
            template_data['belt_parts'].update(create_belt_part_infos(value))
        elif key == 'drops':
            template_data['drops'].update(create_drops_info(value))
        elif key == 'faulty_parts':
            template_data['faulty_parts'].update(create_faulty_parts_info(value))
        elif key == 'orders':
            template_data['orders'].update(create_order_infos(value))
        elif key == 'options':
            pass
        elif key == 'models_to_spawn':
            template_data['models_to_spawn'].update(
                create_models_to_spawn_infos(value))
        elif key == 'time_limit':
            template_data['time_limit'] = value
        else:
            print("Error: unknown top level entry '{0}'".format(key), file=sys.stderr)
            sys.exit(1)
    template_data['bins'] = create_bin_infos()
    template_data['material_locations'] = create_material_location_info(
        template_data['belt_parts'] or {},
        models_over_bins,
    )
    return template_data


def generate_files(template_data):
    files = {}
    for template_file in template_files:
        with open(template_file, 'r') as f:
            data = f.read()
        files[template_file] = em.expand(data, template_data)
    return files


def main(sysargv=None):
    parser = argparse.ArgumentParser(
        description='Prepares and then executes a gazebo simulation based on configurations.')
    prepare_arguments(parser)
    args = parser.parse_args(sysargv)
    config_data = args.config or ''
    if args.file is not None:
        for file in args.file:
            with open(file, 'r') as f:
                comp_config_data = f.read()
                config_data += comp_config_data
    dict_config = yaml.load(config_data) or {}
    expanded_dict_config = expand_yaml_substitutions(dict_config)
    if args.verbose:
        print(yaml.dump({'Using configuration': expanded_dict_config}))
    template_data = prepare_template_data(expanded_dict_config)
    files = generate_files(template_data)
    if not args.dry_run and not os.path.isdir(args.output):
        if os.path.exists(args.output) and not os.path.isdir(args.output):
            print("Error, given output directory exists but is not a directory.", file=sys.stderr)
            sys.exit(1)
        print('creating directory: ' + args.output)
        os.makedirs(args.output)
    for name, content in files.items():
        if name.endswith('.template'):
            name = name[:-len('.template')]
        name = os.path.basename(name)
        if args.dry_run:
            print('# file: ' + name)
            print(content)
        else:
            file_path = os.path.join(args.output, name)
            print('writing file ' + file_path)
            with open(file_path, 'w+') as f:
                f.write(content)
    cmd = [
        #'roslaunch',
        os.path.join(args.output, 'gear.launch'),
        'world_path:=' + os.path.join(args.output, 'gear.world'),
        'gear_urdf_xacro:=' + os.path.join(args.output, 'gear.urdf.xacro'),
    ]
    if args.verbose:
        cmd += ['verbose:=true']
    if args.no_gui:
        cmd += ['gui:=false']

    if not args.development_mode:
        os.environ['ARIAC_COMPETITION'] = '1'
        
    roslaunch_args = cmd[1:] 
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cmd)[0], roslaunch_args)]
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
    roslaunch.configure_logging(uuid)
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    print("Running command: " + ' '.join(cmd))
    if not args.dry_run:
      parent.start(auto_terminate=False)
      parent.spin()
    return 0

   
    #
        #try:
            #p = subprocess.Popen(cmd)
            #p.wait()
        #except KeyboardInterrupt:
            #pass
        #finally:
            #p.wait()
        #return p.returncode


if __name__ == '__main__':
    # Filter out any special ROS remapping arguments.
    # This is necessary if the script is being run from a ROS launch file.
    import rospy
    filtered_argv = rospy.myargv(sys.argv)

    #sys.exit(main(filtered_argv[1:]))
    main(filtered_argv[1:])
