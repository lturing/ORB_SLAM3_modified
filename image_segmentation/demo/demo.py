# ------------------------------------------------------------------------------
# Reference: https://github.com/facebookresearch/Mask2Former/blob/main/demo/demo.py
# Modified by Jitesh Jain (https://github.com/praeclarumjj3)
# ------------------------------------------------------------------------------

import argparse
import multiprocessing as mp
import os
import torch
import random
# fmt: off
import sys
sys.path.insert(1, os.path.join(sys.path[0], '..'))
# fmt: on

import time
import cv2
import numpy as np
import tqdm

from detectron2.config import get_cfg
from detectron2.data.detection_utils import read_image
from detectron2.projects.deeplab import add_deeplab_config
from detectron2.utils.logger import setup_logger
import torch 

from oneformer import (
    add_oneformer_config,
    add_common_config,
    add_swin_config,
    add_dinat_config,
    add_convnext_config,
)
from predictor import VisualizationDemo

# constants
WINDOW_NAME = "OneFormer Demo"

def setup_cfg(args, device):
    # load config from file and command-line arguments
    cfg = get_cfg()
    cfg.MODEL.DEVICE = device
    add_deeplab_config(cfg)
    add_common_config(cfg)
    add_swin_config(cfg)
    add_dinat_config(cfg)
    add_convnext_config(cfg)
    add_oneformer_config(cfg)
    cfg.merge_from_file(args.config_file)
    cfg.merge_from_list(args.opts)
    cfg.freeze()
    return cfg


def get_parser():
    parser = argparse.ArgumentParser(description="oneformer demo for builtin configs")
    parser.add_argument(
        "--config-file",
        default="../configs/ade20k/swin/oneformer_swin_large_IN21k_384_bs16_160k.yaml",
        metavar="FILE",
        help="path to config file",
    )
    parser.add_argument("--task", help="Task type")
    parser.add_argument(
        "--input",
        nargs="+",
        help="A list of space separated input images; "
        "or a single glob pattern such as 'directory/*.jpg'",
    )
    parser.add_argument(
        "--output",
        help="A file or directory to save output visualizations. "
        "If not given, will show output in an OpenCV window.",
    )

    parser.add_argument(
        "--confidence-threshold",
        type=float,
        default=0.5,
        help="Minimum score for instance predictions to be shown",
    )
    parser.add_argument(
        "--opts",
        help="Modify config options using the command-line 'KEY VALUE' pairs",
        default=[],
        nargs=argparse.REMAINDER,
    )
    return parser

if __name__ == "__main__":
    seed = 0
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False

    mp.set_start_method("spawn", force=True)
    args = get_parser().parse_args()
    setup_logger(name="fvcore")
    logger = setup_logger()
    #logger.info("Arguments: " + str(args))

    device = "cuda" if torch.cuda.is_available() else "cpu"

    cfg = setup_cfg(args, device)

    demo = VisualizationDemo(cfg)

    with open('color_map_cityescape.txt', 'w', encoding='utf-8') as f:
        for i in range(len(demo.metadata.stuff_classes)):
            f.write(demo.metadata.stuff_classes[i] + ',' + ' '.join(map(str, demo.metadata.stuff_colors[i])) + '\n')

    if args.input:
        for path in tqdm.tqdm(args.input, disable=not args.output):
            # use PIL, to be consistent with evaluation
            
            img = read_image(path, format="BGR")
            start_time = time.time()
            predictions, visualized_output = demo.run_on_image(img, args.task)
            ''' 
            logger.info(
                "{}: {} in {:.2f}s".format(
                    path,
                    "detected {} instances".format(len(predictions["instances"]))
                    if "instances" in predictions
                    else "finished",
                    time.time() - start_time,
                )
            )
            '''
            if args.output:
                if len(args.input) == 1:
                    for k in visualized_output.keys():
                        os.makedirs(k, exist_ok=True)
                        out_filename = os.path.join(k, args.output)
                        print(out_filename)
                        visualized_output[k].save(out_filename)    
                else:
                    for k in visualized_output.keys():
                        opath = os.path.join(args.output, k)    
                        os.makedirs(opath, exist_ok=True)
                        out_filename = os.path.join(opath, os.path.basename(path))
                        #print(out_filename)
                        visualized_output[k].save(out_filename)  

                        result = predictions["sem_seg"].argmax(dim=0).cpu().detach().numpy().astype(np.uint8)

                        result = result[:, :, 0]
                        opath = os.path.join(args.output + '_color', k)    
                        os.makedirs(opath, exist_ok=True)
                        out_filename = os.path.join(opath, os.path.basename(path))
                        cv2.imwrite(out_filename, result)

            else:
                raise ValueError("Please specify an output path!")
    else:
        raise ValueError("No Input Given")
