sdf_file = '/home/robot/robocon_ws/src/description/worlds/robocon.sdf'
with open(sdf_file, 'r') as f:
    content = f.read()

positions = [
    (1.825, 1.400, 0.375),
    (3.025, 1.400, 0.575),
    (4.225, 1.400, 0.375),
    (1.825, 0.200, 0.575),
    (3.025, 0.200, 0.775),
    (4.225, 0.200, 0.575),
    (1.825, -1.000, 0.775),
    (3.025, -1.000, 0.575)
]

cubes_xml = ""
for i, (x, y, z) in enumerate(positions):
    cubes_xml += f"""
    <model name="kungfu_scroll_{i+1}">
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>0.63</mass>
          <inertia>
            <ixx>0.0128</ixx><iyy>0.0128</iyy><izz>0.0128</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>0.35 0.35 0.35</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.35 0.35 0.35</size></box></geometry>
          <material>
            <ambient>0.9 0.8 0.6 1</ambient>
            <diffuse>0.9 0.8 0.6 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""

if 'kungfu_scroll_1' not in content:
    new_content = content.replace('  </world>', cubes_xml + '\n  </world>')
    with open(sdf_file, 'w') as f:
        f.write(new_content)
    print("Cubes added successfully.")
else:
    print("Cubes already exist.")
