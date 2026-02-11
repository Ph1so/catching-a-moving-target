from PIL import Image
import os

def extract_last_frame(gif_path, output_path):
    with Image.open(gif_path) as im:
        im.seek(im.n_frames - 1)
        last_frame = im.convert("RGB")
        last_frame.save(output_path)
        print(f"Saved {output_path}")

# Process final_g2.gif to final_g12.gif
for i in range(1, 13):
    gif_name = f"final_g{i}.gif"
    output_name = f"final_g{i}.png"

    if os.path.exists(gif_name):
        extract_last_frame(gif_name, output_name)
    else:
        print(f"Skipped {gif_name} (not found)")
