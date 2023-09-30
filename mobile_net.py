import tensorflow as tf
import tensorflow_hub as hub
import time
import numpy as np
from PIL import Image
from PIL import ImageColor
from PIL import ImageDraw
from PIL import ImageFont

import cv2

# Maximum objects to be classified in the image
MAX_OBJECTS = 10


# Labels of interest
# LABEL_SELECTOR = set([b'Person'])

def draw_bounding_box_on_image(image, ymin, xmin, ymax, xmax, color,
                               font, thickness=4, display_str_list=()):
    """Adds a bounding box to an image."""
    draw = ImageDraw.Draw(image)
    im_width, im_height = image.size
    (left, right, top, bottom) = (xmin * im_width, xmax * im_width,
                                  ymin * im_height, ymax * im_height)
    draw.line([(left, top), (left, bottom), (right, bottom), (right, top),
               (left, top)],
              width=thickness,
              fill=color)
    # If the total height of the display strings added to the top of the bounding
    # box exceeds the top of the image, stack the strings below the bounding box
    # instead of above.
    display_str_heights = [font.getsize(ds)[1] for ds in display_str_list]
    # Each display_str has a top and bottom margin of 0.05x.
    total_display_str_height = (1 + 2 * 0.05) * sum(display_str_heights)
    if top > total_display_str_height:
        text_bottom = top
    else:
        text_bottom = bottom + total_display_str_height
    # Reverse list and print from bottom to top.
    for display_str in display_str_list[::-1]:
        text_width, text_height = font.getsize(display_str)
        margin = np.ceil(0.05 * text_height)
        draw.rectangle([(left, text_bottom - text_height - 2 * margin),
                        (left + text_width, text_bottom)],
                       fill=color)
        draw.text((left + margin, text_bottom - text_height - margin),
                  display_str,
                  fill="black",
                  font=font)
        text_bottom -= text_height - 2 * margin


def draw_boxes(image, boxes, class_names, scores, selected_indices, max_boxes=MAX_OBJECTS, min_score=0.3):
    """Overlay labeled boxes on an image with formatted scores and label names."""
    colors = list(ImageColor.colormap.values())
    font = ImageFont.load_default()
    box_count = 0
    for i in range(boxes.shape[0]):
        if box_count >= MAX_OBJECTS:
            break
        if i not in selected_indices:
            continue
        if scores[i] >= min_score:
            ymin, xmin, ymax, xmax = tuple(boxes[i])
            display_str = "{}: {}%".format(class_names[i].decode("ascii"), int(100 * scores[i]))
            color = colors[hash(class_names[i]) % len(colors)]
            image_pil = Image.fromarray(np.uint8(image)).convert("RGB")
            draw_bounding_box_on_image(image_pil, ymin, xmin, ymax, xmax, color, font, display_str_list=[display_str])
            np.copyto(image, np.array(image_pil))
            box_count += 1
    return image, box_count


def get_boxes(image, boxes):
    """Overlay labeled boxes on an image with formatted scores and label names."""
    box_count = 0
    box_lst = []
    for i in range(len(boxes)):
        ymin, xmin, ymax, xmax = tuple(boxes[i])
        im_height, im_width, channel = image.shape
        left, right, top, bottom = (xmin * im_width, xmax * im_width,
                                    ymin * im_height, ymax * im_height)
        box_lst.append((int(left), int(top), int(right - left), int(bottom - top)))
        # box_lst.append((int(left), int(right), int(top), int(bottom)))
        box_count += 1
    return np.array(box_lst)


def non_max_suppression(boxes, scores):
    selected_indices = tf.image.non_max_suppression(boxes, scores, 1000, iou_threshold=0.5,
                                                    score_threshold=float('-inf'), name=None)
    # selected_boxes = tf.gather(boxes, selected_indices)
    return selected_indices.numpy()


class ObjectRecognition:
    def __init__(self):
        self.interpreter = tf.lite.Interpreter(model_path="model_3.tflite")
        self.interpreter.allocate_tensors()

    def run_object_recognition(self, frame):
        converted_img = tf.image.convert_image_dtype(frame, tf.float32)[tf.newaxis, ...]
        result = self.model(converted_img)
        selected_indices = non_max_suppression(result['detection_boxes'], result['detection_scores'])
        result = {key: value.numpy() for key, value in result.items()}

        image_with_boxes, box_count = draw_boxes(
            frame, result["detection_boxes"],
            result["detection_class_entities"], result["detection_scores"], selected_indices)
        return image_with_boxes, box_count

    #
    # def get_boxes(self, frame):
    #     converted_img = tf.image.convert_image_dtype(frame, tf.float32)[tf.newaxis, ...]
    #     result = self.model(converted_img)
    #     selected_indices = non_max_suppression(result['detection_boxes'], result['detection_scores'])
    #     result = {key: value.numpy() for key, value in result.items()}
    #     boxes = get_boxes(
    #         frame, result["detection_boxes"],
    #         result["detection_class_entities"], result["detection_scores"], selected_indices)
    #     return boxes

    def get_boxes(self, frame):
        interpreter = self.interpreter
        image = cv2.resize(frame, (320, 320),interpolation=cv2.INTER_AREA)
        image = image.reshape(1, 320, 320, 3)
        image=np.float32(image)


        # input_tensor = tf.convert_to_tensor(image, dtype=tf.float32)


        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        second = time.time()

        # We use the original model for pre-processing, since the TFLite model doesn't
        # include pre-processing.
        preprocessed_image = (2.0 / 255.0) * image - 1.0

        interpreter.set_tensor(input_details[0]['index'], preprocessed_image)


        interpreter.invoke()
        boxes = interpreter.get_tensor(output_details[1]['index'])
        scores = interpreter.get_tensor(output_details[0]['index'])

        final_boxes = []
        for i in range(len(scores[0])):
            if (scores[0][i] > 0.8):

                final_boxes.append(boxes[0][i])
        final_boxes = get_boxes(frame, final_boxes)


        return final_boxes
