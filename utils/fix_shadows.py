import sys
import re

sdf_file = '/home/robot/robocon_ws/src/description/worlds/robocon.sdf'
with open(sdf_file, 'r') as f:
    content = f.read()

# For all visual blocks named visual_top, visual_front, etc., add <cast_shadows>false</cast_shadows>
# The regex finds <visual name="visual_top"> and adds it right after.

def add_no_shadows(content, visual_name):
    # Use re.sub to insert <cast_shadows>false</cast_shadows> right after the visual tag, 
    # but only if it's not already there.
    
    # regex pattern
    pattern = r'(<visual name="' + visual_name + r'">)'
    replacement = r'\1\n          <cast_shadows>false</cast_shadows>'
    
    # First, let's check if it already has cast_shadows to avoid duplicates
    # A simple way is to replace, but this regex doesn't prevent duplicate.
    # Instead, we will split the content and inject.
    parts = content.split(f'<visual name="{visual_name}">')
    new_content = parts[0]
    for i in range(1, len(parts)):
        if '<cast_shadows>false</cast_shadows>' not in parts[i][:100]:
            new_content += f'<visual name="{visual_name}">\n          <cast_shadows>false</cast_shadows>' + parts[i]
        else:
            new_content += f'<visual name="{visual_name}">' + parts[i]
            
    return new_content

for name in ['visual_top', 'visual_front', 'visual_back', 'visual_left', 'visual_right']:
    content = add_no_shadows(content, name)

with open(sdf_file, 'w') as f:
    f.write(content)

print("Shadows disabled for thin texture planes to fix acne.")
