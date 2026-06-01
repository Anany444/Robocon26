import fitz
import numpy as np
import cv2

doc = fitz.open('KFS Image V1.0.pdf')
for i, page in enumerate(doc):
    pix = page.get_pixmap(alpha=True, dpi=300)
    img = np.frombuffer(pix.samples, dtype=np.uint8).reshape(pix.height, pix.width, pix.n)
    
    if pix.n == 4: # RGBA
        alpha = img[:, :, 3:] / 255.0
        red_bg = np.zeros_like(img[:, :, :3])
        red_bg[:] = [0, 0, 255] # BGR red
        bgr = cv2.cvtColor(img[:, :, :3], cv2.COLOR_RGB2BGR)
        out = (bgr * alpha + red_bg * (1 - alpha)).astype(np.uint8)
    elif pix.n == 3:
        out = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    else:
        out = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # Floodfill to replace opaque white/grey background with RED
    h, w = out.shape[:2]
    mask = np.zeros((h + 2, w + 2), np.uint8)
    
    # We fill from multiple points along the edges to ensure we get the background
    # even if there is a small margin.
    points_to_check = [
        (0, 0), (w//2, 0), (w-1, 0),
        (0, h//2), (w-1, h//2),
        (0, h-1), (w//2, h-1), (w-1, h-1)
    ]
    
    for pt in points_to_check:
        b, g, r = out[pt[1], pt[0]]
        # If it's a light background color (white, light grey)
        if int(b) + int(g) + int(r) > 600:
            flags = 8 | cv2.FLOODFILL_FIXED_RANGE
            cv2.floodFill(out, mask, pt, (0, 0, 255), (15, 15, 15), (15, 15, 15), flags=flags)
            
    out_path = f'/home/robot/robocon_ws/src/description/worlds/kfs_logo_{i+1}.png'
    cv2.imwrite(out_path, out)
    print(f"Saved {out_path}")

print("Processing complete.")
