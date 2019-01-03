from PIL import Image
import os, sys

if len(sys.argv) == 2:
    in_dir = sys.argv[1]

out_dir = "./training_jpg/"

def batch_image(in_dir, out_dir):
    if not os.path.exists(out_dir):
        print(out_dir, "does not exist")
        os.mkdir(out_dir)
    if not os.path.exists(in_dir):
        print(in_dirm, " does not exist")
        return -1
    count = 0
    files = os.listdir(in_dir)
    for pmg_file in files:
        print(os.path.split(pmg_file))
        out_file = os.path.splitext(pmg_file)[0]
        save_path = out_dir + out_file + ".jpg"
        print(pmg_file) 
        img = Image.open(in_dir + pmg_file)
        img.save(save_path)
        

batch_image(in_dir, out_dir)
