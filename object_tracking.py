from mobile_net import ObjectRecognition
from deep_sort.person_id_model.generate_person_features import generate_detections, init_encoder
from deep_sort.deep_sort_app import run_deep_sort, DeepSORTConfig
from deep_sort.fuzzy_util import controller

import numpy as np
from PIL import ImageGrab
import cv2
import drone_kit

import threading
import tkinter as tk
from tkinter import simpledialog


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

selected_object_id = 0

# spawn a new thread to wait for input
def get_target_id():
    ROOT = tk.Tk()

    global selected_object_id
    while(True):
        ROOT.withdraw()
        selected_object_id = simpledialog.askinteger(title="Test",
                                          prompt="Which is your desired target?:")



def object_tracking():
    while (True):
        frame = np.array(ImageGrab.grab(bbox=screen_coordinates))
        # ret, frame = cap.read()
        boxes = model.get_boxes(frame)

        if len(boxes) > 0:
            encoding = generate_detections(encoder, boxes, frame)
            target_box_central=run_deep_sort(frame, encoding, selected_object_id, config)
            print(selected_object_id)
            if(target_box_central!=None):
                fuzzy_result = FuzzyController.run(target_box_central[0], frame.shape[0:2][::-1])
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()


input_target_thread = threading.Thread(target=get_target_id)
tracking_thread = threading.Thread(target=object_tracking)

input_target_thread.start()
tracking_thread.start()

input_target_thread.join()
tracking_thread.join()