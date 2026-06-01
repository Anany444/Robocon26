import sys

sdf_file = '/home/robot/robocon_ws/src/description/worlds/robocon.sdf'
with open(sdf_file, 'r') as f:
    content = f.read()

# We need to split the file by <model name="kungfu_scroll_X"> and replace the occurrences 
# of kfs_logo_1.png in that specific model.

def update_model_logo(content, model_name, new_logo):
    parts = content.split(f'<model name="{model_name}">')
    if len(parts) < 2:
        return content
    
    # Process the model body (up to </model>)
    end_idx = parts[1].find('</model>')
    model_body = parts[1][:end_idx]
    
    # Replace the logo file name in this body
    new_model_body = model_body.replace('kfs_logo_1.png', new_logo)
    
    parts[1] = new_model_body + parts[1][end_idx:]
    return parts[0] + f'<model name="{model_name}">' + parts[1]

# kungfu_scroll_3 already has kfs_logo_1.png
content = update_model_logo(content, "kungfu_scroll_4", "kfs_logo_2.png")
content = update_model_logo(content, "kungfu_scroll_6", "kfs_logo_3.png")

with open(sdf_file, 'w') as f:
    f.write(content)

print("SDF updated with different logos successfully.")
