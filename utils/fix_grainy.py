import sys

sdf_file = '/home/robot/robocon_ws/src/description/worlds/robocon.sdf'
with open(sdf_file, 'r') as f:
    content = f.read()

scene_block = """
    <scene>
      <ambient>0.6 0.6 0.6 1.0</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
      <sky></sky>
    </scene>
"""

# Check if scene block already exists
if "<scene>" not in content:
    # Insert it right before the directional light
    parts = content.split('<light type="directional" name="sun">')
    if len(parts) == 2:
        new_content = parts[0] + scene_block + '    <light type="directional" name="sun">' + parts[1]
        with open(sdf_file, 'w') as f:
            f.write(new_content)
        print("Scene block with sky added successfully.")
    else:
        print("Could not find light tag to insert scene block.")
else:
    print("Scene block already exists.")
