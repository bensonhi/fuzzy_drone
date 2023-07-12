import sys
from mobile_net import ObjectRecognition
from deep_sort.person_id_model.generate_person_features import generate_detections, init_encoder
from deep_sort.deep_sort_app import run_deep_sort, DeepSORTConfig
from deep_sort.application_util.visualization import cv2
import numpy as np
from PIL import ImageGrab
import cv2
import drone_kit
from deep_sort.fuzzy_util import controller

# 設定要捕獲的螢幕範圍（左上角座標和右下角座標）
screen_coordinates = (0, 0, 1920, 1080)  # 自行調整螢幕解析度

drone_kit.arm_and_takeoff(100)
drone_kit.goto(-35.360354, 149.160218, 100)
drone_kit.condition_yaw(240, True, True)
drone_kit.clear()

model = ObjectRecognition()
encoder = init_encoder()
config = DeepSORTConfig()
FuzzyController = controller.FuzzyController()


while (True):
    frame = np.array(ImageGrab.grab(bbox=screen_coordinates))
    # ret, frame = cap.read()
    boxes = model.get_boxes(frame)

    if len(boxes) > 0:
        encoding = generate_detections(encoder, boxes, frame)
        target_box_central = run_deep_sort(frame, encoding, config)
        print(target_box_central)
        if (target_box_central != None):
            fuzzy_result = FuzzyController.run(target_box_central, frame.shape[0:2][::-1])
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
