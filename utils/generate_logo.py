from PIL import Image, ImageDraw, ImageFont
import urllib.request
import os

# Create a red background
img = Image.new('RGB', (512, 512), color=(220, 20, 20))
d = ImageDraw.Draw(img)

# Try to draw a nice looking robocon 2026 logo
# Draw a white rounded rectangle in the middle
d.rounded_rectangle([20, 150, 492, 362], radius=20, fill='white')

# Draw text
# We use default font since we don't know what fonts are installed
try:
    font = ImageFont.truetype("DejaVuSans-Bold.ttf", 60)
except IOError:
    font = ImageFont.load_default()

d.text((40, 180), "ROBOCON", fill='black', font=font)
d.text((180, 260), "2026", fill=(220, 20, 20), font=font)

out_path = '/home/robot/robocon_ws/src/description/worlds/robocon_logo.png'
img.save(out_path)
print(f"Generated {out_path}")
