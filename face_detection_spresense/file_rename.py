import os
import sys
path = './training_data/'

if sys.argv [1] != None:
    rename_path = path + sys.argv[1]


files = os.listdir(rename_path)
i = 1

for file in files:
    os.rename(os.path.join(rename_path, file), os.path.join(rename_path, sys.argv[1] + str(i)+'.jpg'))
    i = i+1
print("DONE")
