# Autodrive Root

This directory is a first-class repository root for autodrive source code. The vehicle runtime target remains `/home/nvidia/AutoDrive`.

## External Model Assets

Large runtime model files are intentionally not versioned in git. A fresh checkout must provision the required weight files into these tracked directories before running the related packages:

- `src/light_recognition/light_recognition/detectLight/weights/`
  Required files include `best_yolov5s_bdd_prew.pt` and `append_model_epoch23_train0.975_val0.981.pth`.
- `src/sign_recognition/sign_recognition/detectSign/weights/`
  Required file includes `weights_2023_2_4.pt`.
- `src/lane_recognition/lane_recognition/detectLane/weights/`
  Required file includes `culane_18.pth`.
- `src/camera_detect/camera_detect/yolov8_sort/weights/`
  Required file includes `ep150-loss2.623-val_loss2.689.pth`.

Provision these files from the team-managed vehicle image or shared model bundle. Do not recommit the binary weight files into this repository.
