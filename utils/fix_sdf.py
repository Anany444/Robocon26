import sys

sdf_file = '/home/robot/robocon_ws/src/description/worlds/robocon.sdf'
with open(sdf_file, 'r') as f:
    content = f.read()

# Fix red color
old_red = """<material>
            <ambient>0.9 0.1 0.1 1</ambient>
            <diffuse>0.9 0.1 0.1 1</diffuse>
          </material>"""
new_red = """<material>
            <ambient>1.0 0.0 0.0 1.0</ambient>
            <diffuse>1.0 0.0 0.0 1.0</diffuse>
            <specular>0.2 0.2 0.2 1.0</specular>
          </material>"""
content = content.replace(old_red, new_red)

# Fix black textures
# Replace package URI with absolute file URI
old_uri = "package://description/worlds/robocon_logo.png"
new_uri = "file:///home/robot/robocon_ws/src/description/worlds/robocon_logo.png"
content = content.replace(old_uri, new_uri)

# Add roughness and metalness to pbr so it doesn't look completely black from metallic lighting
old_pbr = f"<pbr><metal><albedo_map>{new_uri}</albedo_map></metal></pbr>"
new_pbr = f"<pbr><metal><albedo_map>{new_uri}</albedo_map><roughness>0.9</roughness><metalness>0.1</metalness></metal></pbr>"
content = content.replace(old_pbr, new_pbr)

with open(sdf_file, 'w') as f:
    f.write(content)

print("SDF updated successfully.")
