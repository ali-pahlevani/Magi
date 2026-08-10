#!/usr/bin/env python3
"""Derive magi_description/urdf/go2w_body.xacro from the upstream go2w URDF.

Transformations applied:
  * strip the outer <robot> wrapper + livox include (top level owns those)
  * repoint mesh URIs at magi_description/meshes
  * collapse the runs of sibling <material> tags that upstream emits inside a
    single <visual> (only one material per visual is legal URDF) and rename the
    non-ASCII material names to stable ASCII ids
"""
import re

SRC = '/home/maedeh/magi_ws/src/unitree_go2w_ros2/src/go2w_description/urdf/go2w.urdf.xacro'
DST = '/home/maedeh/magi_ws/src/magi_description/urdf/go2w_body.xacro'

t = open(SRC, encoding='utf-8').read()

# --- strip wrapper / livox plumbing -------------------------------------
t = re.sub(r'^\s*<\?xml[^>]*\?>\s*', '', t)
t = re.sub(r'<robot[^>]*>', '', t, count=1)
t = re.sub(r'</robot>\s*$', '', t)
t = re.sub(r'<!--\s*Include Livox.*?-->', '', t, flags=re.S)
t = re.sub(r'<xacro:include[^>]*livox_mid360[^>]*/>', '', t)
t = re.sub(r'<!--\s*Instantiate Livox.*?-->', '', t, flags=re.S)
t = re.sub(r'<xacro:livox_mid360[^>]*/>', '', t)

# --- meshes now live in magi_description --------------------------------
t = t.replace('package://go2w_description/dae/', 'package://magi_description/meshes/')

# --- wheel collisions: primitive cylinder instead of the visual mesh -----
# Upstream reuses left/right_wheel.dae as the collision geometry. A triangle
# mesh rolling on a heightmap gives dartsim badly conditioned contact -- the
# robot drives at roughly a sixth of the commanded speed on the Rubicon terrain
# -- so the collision is replaced by the cylinder the mesh actually describes.
# Dimensions are the mesh bounding box: radius 0.086 (x/z span +/-0.0860) and
# width 0.0518 (y span 0.0222..0.0740), centred 0.0481 out along the axle.
WHEEL_R, WHEEL_W, WHEEL_OFF = 0.086, 0.0518, 0.0481

WHEEL_COLLISION = re.compile(
    r'<collision>\s*<origin[^>]*/>\s*<geometry>\s*'
    r'<mesh filename="package://magi_description/meshes/(left|right)_wheel\.dae"\s*/>\s*'
    r'</geometry>\s*</collision>',
    re.S,
)


def wheel_cylinder(m):
    y = WHEEL_OFF if m.group(1) == 'left' else -WHEEL_OFF
    return (
        '<collision>\n'
        '      <origin rpy="1.5707963267948966 0 0" xyz="0 %.4f 0" />\n'
        '      <geometry>\n'
        '        <cylinder radius="%s" length="%s" />\n'
        '      </geometry>\n'
        '    </collision>' % (y, WHEEL_R, WHEEL_W)
    )


t, n_wheels = WHEEL_COLLISION.subn(wheel_cylinder, t)

# --- one material per visual, ASCII names -------------------------------
MAT = r'<material\s+name="[^"]*"\s*>\s*<color\s+rgba="([^"]+)"\s*/>\s*</material>'
names, order = {}, []


def ascii_name(rgba):
    key = ' '.join('%.6g' % float(v) for v in rgba.split())
    if key not in names:
        names[key] = 'magi_mat_%d' % (len(names) + 1)
        order.append((names[key], key))
    return names[key]


def collapse(m):
    first = re.search(MAT, m.group(0)).group(1)
    n = ascii_name(first)
    return '<material name="%s"><color rgba="%s"/></material>' % (n, ' '.join(
        '%.6g' % float(v) for v in first.split()))


t = re.sub(r'(?:%s\s*)+' % MAT, collapse, t)

header = (
    '<?xml version="1.0"?>\n'
    '<!-- AUTO-DERIVED from unitree_go2w_ros2/go2w_description/urdf/go2w.urdf.xacro\n'
    '     Regenerate with scripts/gen_body.py if upstream changes.\n'
    '     Kinematics and inertials are unchanged from upstream; only the mesh\n'
    '     paths and the illegal repeated material blocks per visual element\n'
    '     were rewritten. -->\n'
    '<robot xmlns:xacro="http://ros.org/wiki/xacro">\n\n'
    '  <!-- material palette (derived from the upstream per-visual colors) -->\n')
for n, rgba in order:
    header += '  <material name="%s"><color rgba="%s"/></material>\n' % (n, rgba)
header += '\n'

open(DST, 'w', encoding='utf-8').write(header + t + '\n</robot>\n')
print('wrote %s' % DST)
print('wheel collisions replaced with cylinders: %d' % n_wheels)
print('materials: %d' % len(order))
for n, c in order:
    print('  %s -> %s' % (n, c))
