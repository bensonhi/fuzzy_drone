from mobile_net import ObjectRecognition
from deep_sort.person_id_model.generate_person_features import generate_detections, init_encoder
from deep_sort.deep_sort_app import run_deep_sort, DeepSORTConfig
from deep_sort.fuzzy_util import controller

import numpy as np
from PIL import ImageGrab
import cv2
# import drone_kit

import threading
import tkinter as tk
from tkinter import simpledialog
import time

# 設定要捕獲的螢幕範圍（左上角座標和右下角座標）
screen_coordinates = (0, 0, 1920, 1080)  # 自行調整螢幕解析度
# #
# drone_kit.arm_and_takeoff(100)
# drone_kit.goto(-35.360354, 149.160218, 100)
# drone_kit.condition_yaw(10, relative=True, clock_wise=True)
# drone_kit.clear()

model = ObjectRecognition()
encoder = init_encoder()
config = DeepSORTConfig()
FuzzyController = controller.FuzzyController()

selected_object_id = -1
target_box_central = None
initial_target_box = None
frame = None


# spawn a new thread to wait for input
def get_target_id():
    ROOT = tk.Tk()

    global selected_object_id
    while (True):
        ROOT.withdraw()
        selected_object_id = simpledialog.askinteger(title="Test",
                                                     prompt="Which is your desired target?:")


def grab_screen():
    global frame
    while (1):
        frame = np.array(ImageGrab.grab(bbox=screen_coordinates))


def object_tracking():
    global initial_target_box, frame
    while (True):
        if (initial_target_box == None) and (frame is not None):
            # ret, frame = cap.read()
            boxes = model.get_boxes(frame)

            if len(boxes) > 0:
                encoding = generate_detections(encoder, boxes, frame)
                initial_target_box = run_deep_sort(frame, encoding, selected_object_id, config)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cv2.destroyAllWindows()


def single_tracker():
    global initial_target_box, frame, target_box_central, selected_object_id
    tracker = cv2.TrackerCSRT.create()
    while (1):
        if (initial_target_box != None) and (frame is not None):
            resized_frame = cv2.resize(frame, (320, 320))
            initial_target_box[0] /= frame.shape[1]
            initial_target_box[1] /= frame.shape[0]
            initial_target_box[2] /= frame.shape[1]
            initial_target_box[3] /= frame.shape[0]
            initial_target_box = [int(i * 320) for i in initial_target_box]

            tracker.init(resized_frame, initial_target_box)
            while (1):

                import time
                start = time.time()
                resized_frame = cv2.resize(frame, (320, 320))

                ok, point = tracker.update(resized_frame)
                if (ok):

                    p1 = [int(point[0]), int(point[1])]
                    p2 = [int(point[0] + point[2]), int(point[1] + point[3])]
                    target_box_central = [(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2]
                    resized_p1 = [int(p1[0] / 320 * 1920), int(p1[1] / 320 * 1080)]
                    resized_p2 = [int(p2[0] / 320 * 1920), int(p2[1] / 320 * 1080)]
                    cv2.rectangle(frame, resized_p1, resized_p2, (0, 0, 255), 3)
                    cv2.imshow("new", frame)
                    if cv2.waitKey(1) & 0xFF == ord('l'):
                        print('stop')
                        initial_target_box = None
                        selected_object_id = -1
                        cv2.destroyAllWindows()
                        break;
                else:
                    print('tracking failed')
                    initial_target_box = None
                    break;
                end = time.time()
                seconds = end - start
                print("Time taken : {0} seconds".format(seconds))


def fuzzy_control():
    while (1):
        if (target_box_central != None):
            fuzzy_result = FuzzyController.run(target_box_central, (320, 320))
            drone_kit.send_attitude_target(0.0, 0.03 * fuzzy_result[0], 0.1 * fuzzy_result[1], 0.0, False, 0.5)
            time.sleep(0.1)


input_target_thread = threading.Thread(target=get_target_id)
grab_screen_thread = threading.Thread(target=grab_screen)
tracking_thread = threading.Thread(target=object_tracking)
fuzzy_thread = threading.Thread(target=fuzzy_control)
single_tracking_thread = threading.Thread(target=single_tracker)

grab_screen_thread.start()
input_target_thread.start()
fuzzy_thread.start()
tracking_thread.start()
single_tracking_thread.start()

grab_screen_thread.join()
input_target_thread.join()
tracking_thread.join()
fuzzy_thread.join()
single_tracking_thread.join()
