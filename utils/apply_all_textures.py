import sys
import random

sdf_file = '/home/robot/robocon_ws/src/description/worlds/robocon.sdf'
with open(sdf_file, 'r') as f:
    content = f.read()

# Assignments
assignments = {
    "kungfu_scroll_1": "kfs_logo_2.png", # Real
    "kungfu_scroll_2": "kfs_logo_5.png", # Fake
    "kungfu_scroll_3": "kfs_logo_1.png", # RBCN
    "kungfu_scroll_4": "kfs_logo_1.png", # RBCN
    "kungfu_scroll_5": "kfs_logo_3.png", # Real
    "kungfu_scroll_6": "kfs_logo_1.png", # RBCN
    "kungfu_scroll_7": "kfs_logo_4.png", # Real
    "kungfu_scroll_8": "kfs_logo_2.png", # Real
}

def generate_texture_blocks(logo_filename):
    texture_uri = f"file:///home/robot/robocon_ws/src/description/worlds/{logo_filename}"
    pbr_material = f"""<material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>{texture_uri}</albedo_map><roughness>0.9</roughness><metalness>0.1</metalness></metal></pbr>
          </material>"""
    
    return f"""
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

def process_model(content, model_name, logo_filename):
    parts = content.split(f'<model name="{model_name}">')
    if len(parts) < 2:
        return content
    
    post_model = parts[1]
    end_idx = post_model.find('</model>')
    model_body = post_model[:end_idx]
    
    # If it already has visual_top, we just replace the file:// URIs
    if '<visual name="visual_top">' in model_body:
        import re
        # Find all albedo_map URIs and replace them
        new_model_body = re.sub(r'file:///home/robot/robocon_ws/src/description/worlds/kfs_logo_\d\.png', 
                                f'file:///home/robot/robocon_ws/src/description/worlds/{logo_filename}', 
                                model_body)
    else:
        # We need to insert the texture blocks after the main <visual name="visual">
        visual_start = model_body.find('<visual name="visual">')
        visual_end = model_body.find('</visual>', visual_start) + len('</visual>')
        
        texture_blocks = generate_texture_blocks(logo_filename)
        new_model_body = model_body[:visual_end] + texture_blocks + model_body[visual_end:]
        
    parts[1] = new_model_body + parts[1][end_idx:]
    return parts[0] + f'<model name="{model_name}">' + parts[1]

for model_name, logo_filename in assignments.items():
    content = process_model(content, model_name, logo_filename)

with open(sdf_file, 'w') as f:
    f.write(content)

print("All models updated with their assigned textures.")
