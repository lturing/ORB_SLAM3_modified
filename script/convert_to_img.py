import numpy as np
import glob 
import tqdm 
import cv2 
import os 



vmin = 1000
vmax = 0.0 

multiply = 25 

out_dir = 'depth_img'
os.makedirs(out_dir, exist_ok=True)


for f in tqdm.tqdm(glob.glob('depth_predicted/*.npy')):
    data = np.load(f)
    assert data.shape[0] == 800 and data.shape[1] == 600
    
    data *= multiply
    v = data.max()
    assert 0.0 <= v <= 255.0
    
    data = data.astype(np.uint8)
    
    v = data.max()
    vmax = max(vmax, v)
    assert 0 <= v <= 255

    path = os.path.join(out_dir, os.path.basename(f).strip('.npy'))
    
    cv2.imwrite(path, data)


print(f'vmax={vmax}')


