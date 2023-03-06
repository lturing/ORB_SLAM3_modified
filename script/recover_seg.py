import cv2 
import numpy as np
import glob 
import os 
import tqdm

colorFile = '/home/spurs/python/OneFormer/color_map_cityescape.txt'
colorImg = '/home/spurs/segment/semantic_result/road_color/semantic_inference'

out_dir = 'seg_img'

height = 60
img = np.zeros((height, 19 * height, 3), dtype=np.uint8)

index = 0 
ns = []
index_to_color = {}
with open(colorFile, 'r', encoding='utf-8') as f:
    for line in f:
        line = line.strip().split(',')
        name = line[0]
        c = list(map(int, line[1].split(' ')))[::-1]
        #c = np.asarray(c[::-1], dtype=np.uint8).reshape(1, 1, 3)
        #img[:, height*index:height*(index+1), :] = c
        
        #cv2.putText(img, name, (height * index, height//3), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 255), 1)
        #ns.append(name)

        index_to_color[index] = c 

        index += 1

#cv2.imshow('img', img)
#cv2.waitKey(0)

out_dir = 'seg_img'
os.makedirs(out_dir, exist_ok=True)

for name in tqdm.tqdm(glob.glob(os.path.join(colorImg, '*.png'))):
    img = cv2.imread(name)
    nimg = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    
    labels, index = np.unique(img[:, :, 0], return_counts=True)
    for label in labels:
        #print(label)
        mask = (img[:, :, 0] == label)
        #print(mask.shape, img.shape)
        
        nimg[mask, :] = index_to_color[label]

    #print(name)
    #cv2.imshow('name', nimg)
    #cv2.waitKey(0)
    
    path = os.path.join(out_dir, os.path.basename(name))
    cv2.imwrite(path, nimg)

