"""Exercise real launch substitution/scoping without starting ROS processes."""
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml
from launch import LaunchContext
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import perform_substitutions, visit_all_entities_and_collect_futures


BRINGUP = Path(__file__).resolve().parents[1]
SIMULATION = BRINGUP.parent.parent / 'simulation' / 'ros'


class LaunchContract(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='uav-launch-')
        self.addCleanup(self.temp.cleanup)
        self.prefix = Path(self.temp.name)
        index = self.prefix / 'share/ament_index/resource_index/packages'
        index.mkdir(parents=True)
        for name, source in [('uav_navigation_bringup', BRINGUP),
                             ('uav_simulation', SIMULATION),
                             ('uav_navigation_ros', None), ('replacement', None),
                             ('mavros', None)]:
            (index / name).touch()
            share = self.prefix / 'share' / name
            if source and source.exists():
                share.symlink_to(source, target_is_directory=True)
            else:
                share.mkdir()
        for package, executables in {
            'uav_navigation_ros': ['odometry_tf_node', 'monocular_depth_node',
                                   'octomap_route_replanner_node',
                                   'local_navigation_node', 'autopilot_adapter_node'],
            'uav_simulation': ['gazebo_receiver'],
            'replacement': ['replacement_depth'],
        }.items():
            for executable in executables:
                path = self.prefix / 'lib' / package / executable
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text('#!/bin/sh\nexit 99\n')
                path.chmod(0o755)
        prefixes = [str(self.prefix), os.environ.get('AMENT_PREFIX_PATH', '')]
        self.env = patch.dict(os.environ, {
            'AMENT_PREFIX_PATH': os.pathsep.join(p for p in prefixes if p)})
        self.env.start()
        self.addCleanup(self.env.stop)

    def run_launch(self, path, **overrides):
        context = LaunchContext()
        context.launch_configurations.update(overrides)
        nodes = []

        def capture(process, active_context):
            command = [perform_substitutions(active_context, arg) for arg in process.cmd]
            name = process.node_name.rsplit('/', 1)[-1]
            files = [command[i + 1] for i, arg in enumerate(command)
                     if arg == '--params-file']
            parameters = {}
            for filename in files:
                content = yaml.safe_load(Path(filename).read_text())
                for key, section in content.items():
                    if key in ('/**', name, '/' + name):
                        parameters.update(section.get('ros__parameters', {}))
            nodes.append(dict(name=name, command=command,
                              parameters=parameters, files=files))
            return []

        description = PythonLaunchDescriptionSource(str(path)).get_launch_description(context)
        with patch.object(ExecuteProcess, 'execute', capture):
            visit_all_entities_and_collect_futures(description, context)
        return nodes, context

    def test_app_does_not_leak_leaf_defaults(self):
        nodes, context = self.run_launch(BRINGUP / 'launch/app.launch.py', start_mavros='false')
        self.assertEqual([Path(n['command'][0]).name for n in nodes], [
            'odometry_tf_node', 'monocular_depth_node', 'octomap_route_replanner_node',
            'local_navigation_node', 'autopilot_adapter_node'])
        self.assertNotIn('node_package', context.launch_configurations)
        self.assertNotIn('node_executable', context.launch_configurations)

    def test_all_disabled_performs_no_process_start(self):
        disabled = {f'start_{name}': 'false' for name in (
            'mavros', 'localization', 'depth', 'global_planner', 'local_planner', 'adapter')}
        nodes, _ = self.run_launch(BRINGUP / 'launch/app.launch.py', **disabled)
        self.assertEqual(nodes, [])

    def test_explicit_depth_parameters_override_yaml_with_correct_types(self):
        params = self.prefix / 'params with spaces.yaml'
        params.write_text('monocular_depth_cpp:\n  ros__parameters:\n'
                          '    use_sim_time: true\n    backend: tensorrt\n'
                          '    model_path: wrong.plan\n')
        nodes, _ = self.run_launch(BRINGUP / 'launch/monocular_depth.launch.py',
                                  params_file=str(params), use_sim_time='false',
                                  model_file='/models/with: spaces.onnx', depth_backend='cpu')
        self.assertEqual(nodes[0]['files'][0], str(params))
        values = nodes[0]['parameters']
        self.assertIs(values['use_sim_time'], False)
        self.assertEqual(values['backend'], 'cpu')
        self.assertEqual(values['model_path'], '/models/with: spaces.onnx')

    def test_same_prior_reaches_global_and_local_guard(self):
        nodes, _ = self.run_launch(BRINGUP / 'launch/app.launch.py', start_mavros='false',
                                  use_sim_time='false', prior_map='/maps/custom prior.bt')
        for name in ('octomap_route_replanner', 'local_navigation'):
            values = next(n['parameters'] for n in nodes if n['name'] == name)
            self.assertEqual(values['prior_map_file'], '/maps/custom prior.bt')
            self.assertIs(values['use_sim_time'], False)

    def test_replacement_launch_only_replaces_depth(self):
        replacement = self.prefix / 'custom_depth.launch.py'
        replacement.write_text(
            'from launch import LaunchDescription\n'
            'from launch.actions import IncludeLaunchDescription\n'
            'from launch.launch_description_sources import PythonLaunchDescriptionSource\n'
            'def generate_launch_description():\n'
            f'    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource({str(BRINGUP / "launch/monocular_depth.launch.py")!r}), '
            'launch_arguments={"node_package": "replacement", '
            '"node_executable": "replacement_depth"}.items())])\n')
        nodes, _ = self.run_launch(BRINGUP / 'launch/app.launch.py', start_mavros='false',
                                  depth_launch_file=str(replacement))
        executables = [Path(n['command'][0]).name for n in nodes]
        self.assertEqual(executables[1], 'replacement_depth')
        self.assertEqual(executables[2], 'octomap_route_replanner_node')

    def test_mavros_wrapper_loads_upstream_xml_and_forwards_connection(self):
        upstream = self.prefix / 'share/mavros/launch/apm.launch'
        upstream.parent.mkdir()
        upstream.write_text(
            '<launch><arg name="fcu_url" default="wrong"/>'
            '<arg name="namespace" default="wrong"/>'
            '<node pkg="replacement" exec="replacement_depth" name="mavros_stub" '
            'namespace="$(var namespace)">'
            '<param name="fcu_url" value="$(var fcu_url)" type="str"/>'
            '</node></launch>')
        nodes, context = self.run_launch(BRINGUP / 'launch/mavros.launch.py',
                                        fcu_url='tcp://192.168.55.100:5762')
        self.assertEqual(nodes[0]['parameters']['fcu_url'], 'tcp://192.168.55.100:5762')
        self.assertEqual(context.launch_configurations['namespace'], '')

    @unittest.skipUnless((SIMULATION / 'launch').exists(), 'Simulation not copied into this workspace')
    def test_forest_receiver_scope_and_typed_port(self):
        nodes, _ = self.run_launch(SIMULATION / 'launch/forest.launch.py', start_mavros='false',
                                  receiver_bind_address='127.0.0.1', receiver_port='7449',
                                  params_file=str(BRINGUP / 'config/runtime.yaml'))
        self.assertEqual(Path(nodes[0]['command'][0]).name, 'gazebo_receiver')
        self.assertEqual(nodes[0]['parameters'], {'bind_address': '127.0.0.1', 'port': 7449})
        self.assertEqual(Path(nodes[1]['command'][0]).name, 'odometry_tf_node')
        self.assertEqual(len(nodes), 6)


if __name__ == '__main__':
    unittest.main()
