import re

sdf_file = '/home/robot/robocon_ws/src/description/worlds/robocon.sdf'
with open(sdf_file, 'r') as f:
    content = f.read()

def update_model_logo_uri(content, model_name, new_uri):
    parts = content.split(f'<model name="{model_name}">')
    if len(parts) < 2:
        return content
    
    post_model = parts[1]
    end_idx = post_model.find('</model>')
    model_body = post_model[:end_idx]
    
    # Replace anything between <albedo_map> and </albedo_map> with the new URI
    new_model_body = re.sub(r'<albedo_map>.*?</albedo_map>', f'<albedo_map>{new_uri}</albedo_map>', model_body)
    
    parts[1] = new_model_body + parts[1][end_idx:]
    return parts[0] + f'<model name="{model_name}">' + parts[1]

assignments = {
    "kungfu_scroll_1": "file:///home/robot/robocon_ws/src/description/textures/r2/real_kfs/kfs_logo_17.png",
    "kungfu_scroll_2": "file:///home/robot/robocon_ws/src/description/textures/r2/fake_kfs/kfs_logo_8.png",
    "kungfu_scroll_3": "file:///home/robot/robocon_ws/src/description/textures/r1/robocon_logo_red_bg.png",
    "kungfu_scroll_4": "file:///home/robot/robocon_ws/src/description/textures/r1/robocon_logo_red_bg.png",
    "kungfu_scroll_5": "file:///home/robot/robocon_ws/src/description/textures/r2/real_kfs/kfs_logo_21.png",
    "kungfu_scroll_6": "file:///home/robot/robocon_ws/src/description/textures/r1/robocon_logo_red_bg.png",
    "kungfu_scroll_7": "file:///home/robot/robocon_ws/src/description/textures/r2/real_kfs/kfs_logo_29.png",
    "kungfu_scroll_8": "file:///home/robot/robocon_ws/src/description/textures/r2/real_kfs/kfs_logo_30.png",
}

for model_name, new_uri in assignments.items():
    content = update_model_logo_uri(content, model_name, new_uri)

with open(sdf_file, 'w') as f:
    f.write(content)

print("Paths successfully updated.")
