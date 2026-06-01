import re

sdf_file = '/home/robot/robocon_ws/src/description/worlds/robocon.sdf'
with open(sdf_file, 'r') as f:
    content = f.read()

# Make all boxes red by replacing their material
old_material = """<material>
            <ambient>0.9 0.8 0.6 1</ambient>
            <diffuse>0.9 0.8 0.6 1</diffuse>
          </material>"""
new_material = """<material>
            <ambient>0.9 0.1 0.1 1</ambient>
            <diffuse>0.9 0.1 0.1 1</diffuse>
          </material>"""
content = content.replace(old_material, new_material)

# Now, target boxes 3, 4, 6 and replace their visual block to include textures
# We need to find the visual block for those specific models.

def add_texture_to_model(content, model_name):
    # Find the visual block of the model
    # A bit tricky with regex, let's do string splitting
    parts = content.split(f'<model name="{model_name}">')
    if len(parts) < 2:
        return content
    
    post_model = parts[1]
    visual_start = post_model.find('<visual name="visual">')
    visual_end = post_model.find('</visual>', visual_start) + len('</visual>')
    
    old_visual = post_model[visual_start:visual_end]
    
    texture_uri = "package://description/worlds/robocon_logo.png"
    pbr_material = f"""<material>
            <pbr><metal><albedo_map>{texture_uri}</albedo_map></metal></pbr>
          </material>"""
    
    new_visuals = old_visual + f"""
        <!-- Top -->
        <visual name="visual_top">
          <pose>0 0 0.176 0 0 0</pose>
          <geometry><box><size>0.35 0.35 0.001</size></box></geometry>
          {pbr_material}
        </visual>
        <!-- Front -->
        <visual name="visual_front">
          <pose>0.176 0 0 0 0 0</pose>
          <geometry><box><size>0.001 0.35 0.35</size></box></geometry>
          {pbr_material}
        </visual>
        <!-- Back -->
        <visual name="visual_back">
          <pose>-0.176 0 0 0 0 0</pose>
          <geometry><box><size>0.001 0.35 0.35</size></box></geometry>
          {pbr_material}
        </visual>
        <!-- Left -->
        <visual name="visual_left">
          <pose>0 0.176 0 0 0 0</pose>
          <geometry><box><size>0.35 0.001 0.35</size></box></geometry>
          {pbr_material}
        </visual>
        <!-- Right -->
        <visual name="visual_right">
          <pose>0 -0.176 0 0 0 0</pose>
          <geometry><box><size>0.35 0.001 0.35</size></box></geometry>
          {pbr_material}
        </visual>"""
    
    parts[1] = post_model[:visual_start] + new_visuals + post_model[visual_end:]
    return parts[0] + f'<model name="{model_name}">' + parts[1]

content = add_texture_to_model(content, "kungfu_scroll_3")
content = add_texture_to_model(content, "kungfu_scroll_4")
content = add_texture_to_model(content, "kungfu_scroll_6")

with open(sdf_file, 'w') as f:
    f.write(content)

print("SDF updated successfully.")
