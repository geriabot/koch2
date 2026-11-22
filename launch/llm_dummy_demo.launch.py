#!/usr/bin/env python3

# Copyright 2025 Rodrigo Pérez-Rodríguez
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

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Keys of <component>_llm_options and their types (parameters of the LLM nodes)
PLANNER_OPTIONS = {'url': str, 'num_ctx': int, 'timeout_sec': float, 'keep_alive': str,
                   'think': str, 'max_tokens': int, 'temperature': float}
BT_BUILDER_OPTIONS = {'url': str, 'num_ctx': int, 'timeout_sec': float, 'keep_alive': str,
                      'think': str, 'max_tokens': int, 'max_history_turns': int,
                      'repair_restart_after': int, 'feedback_level': int,
                      'generation_mode': str, 'dataflow_max_attempts': int}


def _llm_options(config, component):
    """llm_options of the config overridden by <component>_llm_options, typed."""
    types = PLANNER_OPTIONS if component == 'planner' else BT_BUILDER_OPTIONS
    shared = config.get('llm_options') or {}
    own = config.get(f'{component}_llm_options') or {}
    merged = {k: v for k, v in shared.items() if k in types}   # shared: what applies
    merged.update(own)
    known = set(PLANNER_OPTIONS) | set(BT_BUILDER_OPTIONS)
    for key in shared:
        if key not in known:
            raise ValueError(f'llm_options: unknown key "{key}" (known: {", ".join(sorted(known))})')
    options = {}
    for key, value in merged.items():
        if key not in types:
            raise ValueError(f'{component}_llm_options: unknown key "{key}" '
                             f'(known: {", ".join(types)})')
        if types[key] is str and isinstance(value, bool):
            value = 'true' if value else 'false'   # think: false in YAML is a bool
        options[key] = types[key](value)
    return options


def launch_setup(context, *args, **kwargs):
    """Read the unified config YAML and build the node list from it.

    All LLM provider settings, BT parameters, goal, and context are taken
    directly from the config file so that every mode (llm / fixed) is
    configured through a single YAML with the same structure.

    The 'launch_llm_nodes' argument controls whether llm_planner and
    llm_bt_builder are started here (true) or are assumed to be already
    running externally (false).
    """
    config_file     = LaunchConfiguration('config_file').perform(context)
    skills_file     = LaunchConfiguration('skills_file').perform(context)
    launch_llm_nodes = LaunchConfiguration('launch_llm_nodes').perform(context).lower() == 'true'
    save_exec = LaunchConfiguration('save_exec').perform(context).lower() == 'true'
    replan_active = LaunchConfiguration('replan_active').perform(context)

    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    # ── LLM provider ─────────────────────────────────────────────────────────
    planner_provider = config.get('planner_llm_provider', config.get('llm_provider', 'gemini'))
    planner_model    = config.get('planner_llm_model',    config.get('llm_model',    'gemini-2.5-flash'))
    planner_api_key  = config.get('planner_llm_api_key',  config.get('llm_api_key',  ''))

    bt_builder_provider = config.get('bt_builder_llm_provider', config.get('llm_provider', 'gemini'))
    bt_builder_model    = config.get('bt_builder_llm_model',    config.get('llm_model',    'gemini-2.5-flash'))
    bt_builder_api_key  = config.get('bt_builder_llm_api_key',  config.get('llm_api_key',  ''))

    # Options of the LLM call (endpoint, and for local models served by Ollama: context,
    # reasoning, timeout...). See the comments of llm_local_config.yaml
    planner_options    = _llm_options(config, 'planner')
    bt_builder_options = _llm_options(config, 'bt_builder')

    # ── BT executor ──────────────────────────────────────────────────────────
    bt_nodes_package      = config.get('bt_nodes_package',    'dummy_bt_nodes')
    bt_control_period_ms  = config.get('bt_control_period_ms', 50)
    bt_timeout_sec        = config.get('bt_timeout_sec',        60.0)

    plugin_libraries = config.get('plugin_libraries', [])
    plugin_library   = plugin_libraries[0] if plugin_libraries else 'libdummy_bt_nodes_plugin.so'

    nodes = []

    # ── 1. LLM Planner node (optional) ───────────────────────────────────────
    if launch_llm_nodes:
        nodes.append(Node(
            package='llm_planner',
            executable='llm_planner_node',
            name='llm_planner_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'llm_provider': planner_provider,
                'llm_model_id': planner_model,
                'llm_api_key':  planner_api_key,
                **{f'llm_{k}' if k != 'url' else 'llm_api_url': v
                   for k, v in planner_options.items() if k in PLANNER_OPTIONS},
            }],
        ))

    # ── 2. LLM BT builder node (optional) ────────────────────────────────────
    if launch_llm_nodes:
        nodes.append(Node(
            package='llm_bt_builder',
            executable='bt_rag_agent_node.py',
            name='llm_bt_agent',
            output='screen',
            emulate_tty=True,
            # The parameters of bt_rag_agent_node are model_id, api_key and api_url (not llm_*)
            parameters=[{
                'llm_provider': bt_builder_provider,
                'model_id':     bt_builder_model,
                'api_key':      bt_builder_api_key,
                **{('api_url' if k == 'url' else k): v
                   for k, v in bt_builder_options.items() if k in BT_BUILDER_OPTIONS},
            }],
        ))

    # ── 3. Action Executor (handles both fixed and LLM orchestrators) ─────────
    executor_args = [config_file]
    if save_exec:
        executor_args.append('--save-exec')
    executor_args.extend(['--replan-active', replan_active])
    nodes.append(Node(
        package='koch2',
        executable='mission_executor',
        name='mission_executor',
        output='screen',
        emulate_tty=True,
        arguments=executor_args,
    ))

    # ── 4. Goal sender (reads goal + context from the same config file) ───────
    nodes.append(TimerAction(
        period=3.0,
        actions=[Node(
            package='koch2',
            executable='start_mission',
            name='start_mission',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'mission_file': config_file,
                'skills_file': skills_file,
            }],
        )],
    ))

    return nodes


def generate_launch_description():
    """Launch the full LLM planning + execution pipeline.

    Nodes started:
      1. llm_planner_node    — generates and replans task YAML via LLM        (optional)
      2. llm_bt_agent_node   — generates BT XML from a step description via LLM (optional)
      3. mission_executor    — runs the orchestrator of the config (planning, BT generation, execution)
      4. start_mission       — sends the initial mission (after a 3 s delay)

    All configuration (LLM provider/model/key, BT parameters, goal, context)
    is loaded from a single YAML file using the unified format shared with the
    fixed-orchestrator configs (simple_config.yaml, restaurant_config.yaml).

    Set launch_llm_nodes:=false when llm_planner and llm_bt_builder are
    already running in another terminal / launch file.
    """
    pkg_dir = get_package_share_directory('koch2')
    default_config = os.path.join(pkg_dir, 'config', 'llm_config.yaml')
    default_skills = os.path.join(pkg_dir, 'config', 'skills.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description=(
            'Path to the unified YAML config file '
            '(orchestrator_type must be "llm")'
        ),
    )

    skills_file_arg = DeclareLaunchArgument(
        'skills_file',
        default_value=default_skills,
        description='Path to the YAML file listing the robot skills (skills: [...]).',
    )

    launch_llm_nodes_arg = DeclareLaunchArgument(
        'launch_llm_nodes',
        default_value='false',
        description=(
            'Whether to launch llm_planner and llm_bt_builder nodes. '
            'Set to "false" if they are already running externally.'
        ),
    )

    save_exec_arg = DeclareLaunchArgument(
        'save_exec',
        default_value='false',
        description=(
            'Save generated BT XMLs to exec/<mission_name>/<timestamp>/. '
            'Set to "true" to enable execution logging.'
        ),
    )

    replan_active_arg = DeclareLaunchArgument(
        'replan_active',
        default_value='true',
        description=(
            'Enable mission replanning after execution/generation failures. '
            'Set to "false" to disable replans (FixBT remains enabled).'
        ),
    )

    return LaunchDescription([
        config_file_arg,
        skills_file_arg,
        launch_llm_nodes_arg,
        save_exec_arg,
        replan_active_arg,
        OpaqueFunction(function=launch_setup),
    ])
