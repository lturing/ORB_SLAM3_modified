import torch 
from PIL import Image
import tqdm 
import numpy as np
from zoedepth.models.builder import build_model
from zoedepth.utils.config import get_config
from zoedepth.utils.misc import save_raw_16bit
from zoedepth.utils.misc import colorize

import os
import glob 
import cv2 


# ZoeD_N(for indoor)
# conf = get_config("zoedepth", "infer")

# ZoeD_N(for outdoor)
conf = get_config("zoedepth", "infer", config_version="kitti")
print(conf)
#conf['save_dir'] = '/home/spurs/.cache/torch/hub/checkpoints'

# zoeD_M12_N for indoor
# conf['pretrained_resource'] = 'local::./ZoeD_M12_N.pt'

# zoeD_M12_K for outdoor 
conf['pretrained_resource'] = 'local::./ZoeD_M12_K.pt'
print(conf)

model_zoe = build_model(conf)
#model_zoe_n = torch.hub.load(".", "ZoeD_N", source="local", pretrained=True)


DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
zoe = model_zoe.to(DEVICE)

img_dir = '/home/spurs/dataset/30fps/2023_02_21_14_04_08/data/img_small/*.png'
for img in tqdm.tqdm(glob.glob(img_dir)):
    image = Image.open(img).convert("RGB")  # load
    #depth_numpy = zoe.infer_pil(image)  # as numpy

    #depth_pil = zoe.infer_pil(image, output_type="pil")  # as 16-bit PIL Image
    # https://github.com/isl-org/ZoeDepth/issues/10
    depth_tensor = zoe.infer_pil(image, output_type="tensor", pad_input=False).cpu().detach().numpy()  # as torch tensor

    
    #print(depth_tensor.shape)
    #print(depth_tensor.dtype, depth_tensor.min(), depth_tensor.max())

    fpath = "output.png"
    fpath = os.path.join('output')
    os.makedirs(fpath, exist_ok=True)
    fpath = os.path.join(fpath, os.path.basename(img))

    np.save(fpath + '.npy',  depth_tensor)

    ''' 
    save_raw_16bit(depth_tensor, fpath)

    image = Image.open(img).convert("L") 
    image = np.asarray(image)

    print(image.shape, image.min(), image.max())

    #cv2.imwrite(fpath, depth_tensor)
    #colored = colorize(depth_tensor)
    #colored = colorize(depth_tensor, 0, 10)
    #Image.fromarray(colored).save(fpath)

    '''
