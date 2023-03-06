import cv2 
import os 
import glob 
import numpy as np
import tqdm 


img_dir = '/home/spurs/dataset/30fps/2023_02_21_14_04_08/data/img'
seg_dir_1 = '/home/spurs/segment/semantic_result/road/semantic_inference'
seg_dir_2 = '/home/spurs/segment/semantic_result/road_color/semantic_inference'

imgs = glob.glob(os.path.join(img_dir, '*.png'))
imgs.sort()

index = 0 
out_dir = 'output'
os.makedirs(out_dir, exist_ok=True)

for f in tqdm.tqdm(imgs):
    name = os.path.basename(f)
    img = os.path.join(img_dir, name)
    img = cv2.imread(img)

    seg1 = os.path.join(seg_dir_1, name)
    seg1 = cv2.imread(seg1)

    seg2 = os.path.join(seg_dir_2, name)
    #seg2 = cv2.imread(seg2)

    height = img.shape[0] #+ seg1.shape[0] #+ seg2.shape[0]
    width = img.shape[1] + seg1.shape[1] #+ seg2.shape[1]

    nimg = np.zeros((height, width, 3), np.uint8)
    nimg[:, :img.shape[1], :] = img 
    
    nimg[:, img.shape[1]:img.shape[1]+seg1.shape[1], :] = seg1
    
    if False:
        cv2.imshow("myImage", nimg)
        cv2.waitKey(1)
    
    name = os.path.join(out_dir, str(index).zfill(5) + '.png')
    cv2.imwrite(name, nimg)
    
    index += 1

