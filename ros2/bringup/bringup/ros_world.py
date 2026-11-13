"""Derives a ROS-ready copy of a Webots world without touching the original.

The copy is identical except that the ME480Robot node uses an <extern>
controller, so webots_ros2_driver can attach to it. The original world keeps
its own controller and still runs stand-alone in Webots.
"""

import os
import re
import tempfile

ROBOT_PROTO = 'ME480Robot'
DEFAULT_ROBOT_NAME = 'robot'   # default of the PROTO's "name" field

_ROBOT_NODE = re.compile(
    r'^((?:DEF \w+ )?' + ROBOT_PROTO + r' \{\n)(.*?)(^\})', re.MULTILINE | re.DOTALL)
_ASSET_URL = re.compile(r'"([^"]*\.(?:jpe?g|png|hdr|obj|stl|dae|wav|mp3|proto))"')


def make_ros_world(world_path):
    """Write the derived world to a temp file and return (path, robot name)."""
    world_path = os.path.abspath(world_path)
    world_dir = os.path.dirname(world_path)
    with open(world_path) as world_file:
        content = world_file.read()

    nodes = _ROBOT_NODE.findall(content)
    if len(nodes) != 1:
        raise RuntimeError(
            f'Expected exactly one {ROBOT_PROTO} node in {world_path}, found {len(nodes)}. '
            f'Add it in Webots (Add node > PROTO nodes (Current Project) > {ROBOT_PROTO}) and save.')

    robot_name = DEFAULT_ROBOT_NAME

    def use_extern_controller(match):
        nonlocal robot_name
        fields = match.group(2)
        name = re.search(r'^\s*name "([^"]*)"', fields, re.MULTILINE)
        if name:
            robot_name = name.group(1)
        fields = re.sub(r'^\s*controller "[^"]*"\n', '', fields, flags=re.MULTILINE)
        return match.group(1) + fields + '  controller "<extern>"\n' + match.group(3)

    content = _ROBOT_NODE.sub(use_extern_controller, content)

    # The copy lives in another directory, so relative asset paths must become absolute.
    def absolute_url(match):
        url = match.group(1)
        if os.path.isabs(url) or '://' in url:
            return match.group(0)
        return '"' + os.path.normpath(os.path.join(world_dir, url)) + '"'

    content = _ASSET_URL.sub(absolute_url, content)

    stem = os.path.splitext(os.path.basename(world_path))[0]
    with tempfile.NamedTemporaryFile(
            mode='w', prefix=stem + '_ros_', suffix='.wbt', delete=False) as ros_world:
        ros_world.write(content)
    return ros_world.name, robot_name
