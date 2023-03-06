import glob
import os
import cv2 
import matplotlib.pyplot as plt 
import tqdm 
from multiprocessing import Pool


def saveimg(data):
    f = data['f']
    index = data['index']

    dimg = cv2.imread(f)

    img = cv2.imread(os.path.join(img_dir, os.path.basename(f)))[:,:,::-1]

    assert dimg.shape[0] == 800 and dimg.shape[1] == 600 
    assert img.shape[0] == 800 and img.shape[1] == 600 
        
    # 800 * 1200
    fig = plt.figure(figsize=(10,10))
    gs = fig.add_gridspec(1, 2, hspace=0., wspace=0.)
    axis = gs.subplots(sharex=True, sharey=True)

    #figure, axis = plt.subplots(1, 2)
        
    axis[0].imshow(img)
    axis[0].axis('off')

    axis[1].imshow(dimg, cmap='gray')
    axis[1].axis('off')
    plt.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=0, hspace=0)

    path = os.path.join(out_dir, str(index).zfill(5) + '.png')
    fig.savefig(path, bbox_inches='tight', pad_inches = 0)

    plt.close()


if __name__ == '__main__':
    paras = []
    img_dir = '/home/spurs/dataset/30fps/2023_02_21_14_04_08/data/img'
    depth_dir = '/home/spurs/segment/depth_img'


    files = glob.glob(os.path.join(depth_dir, '*.png'))
    files.sort()

    out_dir = 'rgb_depth'
    os.makedirs(out_dir, exist_ok=True)
    
    for i, f in enumerate(files):
        data = {'f': f, 'index': i}
        paras.append(data)


    pool = Pool(processes=12) 
    pool.map(saveimg, paras) 
    pool.close()
    pool.join()




