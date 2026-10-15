# KC Task2 arrow recognition API

Task2-only backend using the trained YOLO11m LEFT/RIGHT checkpoint from:
https://github.com/PranavU-Coder/arrow_detection/tree/yolo
Pinned commit335c0dc275769edd902c5bb41a99992fa12fff9c; original file runs/detect/train/weights/best.pt. Checkpoint SHA256 is enforced at startup in main.py. Source repository LICENSE governs its weights; consult the upstream license before redistribution. No model weights were pushed to the group repository.

## Start on Kia Chun's laptop

Stop the prior image API in its own terminal with Ctrl+C if it owns port8001. Do not stop unrelated processes or run this alongside another API on the same port.

```sh
cd "/Users/kiachun/Documents/ChatGPT/MDP ChatGPT"
./KC_start_task2_arrow_api.sh
```

The launcher uses the existing image-API virtual environment and starts0.0.0.0:8001. It keeps the model loaded/warmed between requests. It does not launch the Algo server, connect to STM32 or access the Pi camera. The older KC_start_latest_laptop.sh remains the general group launcher; it does not select this new Task2 backend. Do not use that launcher concurrently on8001.

Check http://localhost:8001/health for best_yolo11m_arrows.pt and total_classes2. Service will report binding failure if8001 is already occupied; it never kills the owner automatically.

## Existing native Pi bridge remains compatible

No firmware flash or Pi file upload required. When Pi is available, use the same laptop IP and full command:

```sh
cd ~/SC2079-MDP/rpi/task2/KC_STM32_Task2_Trial
python3 KC_task2_stm32_bridge.py --execute --stage full --port /dev/ttyACM0 --laptop-ip 192.168.50.218
```

If the laptop address changes, replace it. Native bridge posts multipart image to8001/predict. API returns image_id LEFT/RIGHT, conf and mapped symbol ID39/38. Acceptance stays50%; lower scores return NA and preserve the best candidate/raw detections for diagnosis. Select highest arrow confidence, not largest bounding box. No fallback/guessed direction is sent when no arrow passes. Model inference is serialized; original uploaded bytes and full result JSON are saved under this folder's own_results. PIL RGB input is used directly. Bad input returns400/413; valid empty detections return200/NA. API does not change STM32 sensor or motion behavior.

## Rollback / Task1

Stop the arrow API with Ctrl+C, then run ../KC_start_task2_v11i_rollback.sh to restore the former31-class image API on8001. For Task1, use the original group launcher/model setup. The arrow model only recognises LEFT and RIGHT and is unsuitable as the Task1 symbol recogniser. Original KC_image_api_v11i/main.py and model weights were not edited.

## Validation and limits

Tested locally using FastAPI's multipart HTTP test client against the REAL model: all eight saved RIGHT O2 photos passed50% (85.17–91.02%), matching offline results. Tests cover low confidence, exact threshold, empty detection list, large low-score box versus actual arrow, correct class ID mapping, invalid uploads and upload size limit. No Pi was available and no live camera/robot run was performed. LEFT arrows, O1 frames and background false positives still require physical testing. Upstream author reports false positives;8selected all-RIGHT images are not a full evaluation.

Run tests with the launcher's existing venv Python:
`python -m unittest discover -s KC_task2_arrow_api -p test_api.py` from the parent folder.

Model is loaded via restricted PyTorch weights-only deserialization using known installed layer classes. It uses CPU; no repository Python code is executed. Existing installed runtime: PyTorch2.14.0, Ultralytics8.4.162. On another computer install a compatible environment with PyTorch>=2.6, Ultralytics supporting YOLO11, FastAPI, uvicorn, python-multipart and Pillow, and set KC_TASK2_PYTHON to that interpreter.
