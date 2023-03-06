export task=panoptic
#export task=instance
export task=semantic

export config=configs/ade20k/dinat/coco_pretrain_oneformer_dinat_large_bs16_160k_1280x1280.yaml
export ckpt=coco_pretrain_1280x1280_150_16_dinat_l_oneformer_ade20k_160k.pth

export config=configs/cityscapes/convnext/mapillary_pretrain_oneformer_convnext_large_bs16_90k.yaml
export ckpt=mapillary_pretrain_250_16_convnext_l_oneformer_cityscapes_90k.pth

CUDA_VISIBLE_DEVICES='' python demo/demo.py --config-file $config \
  --input /home/spurs/dataset/30fps/2023_02_21_14_04_08/data/img/*.png \
  --output road \
  --task $task \
  --opts MODEL.IS_TRAIN False MODEL.IS_DEMO True MODEL.WEIGHTS $ckpt
