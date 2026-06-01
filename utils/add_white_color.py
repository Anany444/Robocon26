import sys

sdf_file = '/home/robot/robocon_ws/src/description/worlds/robocon.sdf'
with open(sdf_file, 'r') as f:
    content = f.read()

# We need to find all <material> blocks that have <pbr> but no <ambient>
# And add <ambient>1 1 1 1</ambient> <diffuse>1 1 1 1</diffuse>

old_pbr = "<pbr>"
new_pbr = "<ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr>"
content = content.replace(old_pbr, new_pbr)

with open(sdf_file, 'w') as f:
    f.write(content)

print("SDF updated successfully.")
