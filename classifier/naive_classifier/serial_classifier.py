#!/usr/bin/env python3
"""Learned trash detector: SSDLite320 + MobileNetV3, trained by Oliver Ye.

Drop-in replacement for the HSV detector in classifier/src/classifier.cpp. Reads
JPEG frames from the camera process on tcp :5553 and publishes the target centroid
as a CommandPoint {float x, y, z} on tcp :5555. Coordinates are pixels in the
camera's 300x300 preview frame, and (-1, -1, 0) means no target is in view.
"""
import argparse
import struct
from pathlib import Path

import cv2
import numpy as np
import torch
import torchvision
import torchvision.transforms as transforms
import zmq
from PIL import Image

from timerStream import TimerStream

FRAME_ENDPOINT = "tcp://localhost:5553"
TARGET_ENDPOINT = "tcp://*:5555"
COMMAND_POINT_FORMAT = "fff"
NO_TARGET_CENTROID = (-1.0, -1.0)

MODEL_INPUT_SIZE = 320
DEFAULT_WEIGHTS_PATH = Path(__file__).parent / "weights" / "weights0.05_64_30_27"

# Keep steering at the last detection until this many frames in a row come up empty,
# since the detector flickers between frames.
MAX_FRAMES_WITHOUT_DETECTION = 10


def load_model(weights_path, torch_device):
    # weights_backbone must match training: passing None builds a smaller backbone
    # that the saved state dict does not fit.
    model = torchvision.models.detection.ssdlite.ssdlite320_mobilenet_v3_large(
        weights_backbone="DEFAULT", num_classes=2
    )
    model.load_state_dict(torch.load(weights_path, map_location=torch_device))
    return model.to(torch_device).eval()


def build_image_transforms():
    return transforms.Compose([
        transforms.Resize((MODEL_INPUT_SIZE, MODEL_INPUT_SIZE)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])


def model_inference(bgr_frame, torch_device, image_transforms, model, threshold):
    """Return detection boxes scoring above threshold, best first, in model input pixels."""
    rgb_image = Image.fromarray(bgr_frame[:, :, [2, 1, 0]])
    image_tensor = image_transforms(rgb_image).to(torch_device)
    with torch.no_grad():
        detections = model([image_tensor])[0]
    return detections["boxes"][detections["scores"] > threshold]


def box_centroid_in_frame(box, frame_shape):
    """Scale a box from model input pixels to frame pixels and return its center."""
    frame_height, frame_width = frame_shape[:2]
    x_min, y_min, x_max, y_max = box.tolist()
    center_x = (x_min + x_max) / 2 * frame_width / MODEL_INPUT_SIZE
    center_y = (y_min + y_max) / 2 * frame_height / MODEL_INPUT_SIZE
    return center_x, center_y


class TargetTracker:
    """Holds the last detection across short gaps in the detector's output."""

    def __init__(self, max_frames_without_detection=MAX_FRAMES_WITHOUT_DETECTION):
        self.max_frames_without_detection = max_frames_without_detection
        self.frames_without_detection = 0
        self.centroid = NO_TARGET_CENTROID

    def update(self, detected_centroid):
        if detected_centroid is not None:
            self.centroid = detected_centroid
            self.frames_without_detection = 0
            return self.centroid

        self.frames_without_detection += 1
        if self.frames_without_detection > self.max_frames_without_detection:
            self.centroid = NO_TARGET_CENTROID
        return self.centroid


def open_frame_subscriber(context):
    frame_subscriber = context.socket(zmq.SUB)
    # Inference is slower than the camera, so keep only the newest frame. CONFLATE
    # has to be set before connecting.
    frame_subscriber.setsockopt(zmq.CONFLATE, 1)
    frame_subscriber.connect(FRAME_ENDPOINT)
    frame_subscriber.setsockopt(zmq.SUBSCRIBE, b"")
    return frame_subscriber


def open_target_publisher(context):
    target_publisher = context.socket(zmq.PUB)
    target_publisher.bind(TARGET_ENDPOINT)
    return target_publisher


def decode_frame(jpeg_bytes):
    return cv2.imdecode(np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)


def publish_centroid(target_publisher, centroid):
    center_x, center_y = centroid
    message = struct.pack(COMMAND_POINT_FORMAT, center_x, center_y, 0.0)
    target_publisher.send(message, zmq.DONTWAIT)


def show_frame(frame, centroid):
    if centroid != NO_TARGET_CENTROID:
        cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 5, (0, 255, 0), -1)
    cv2.imshow("Learned classifier", frame)
    cv2.waitKey(1)


def run(weights_path, threshold, show):
    torch_device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {torch_device}")
    model = load_model(weights_path, torch_device)
    image_transforms = build_image_transforms()

    context = zmq.Context()
    frame_subscriber = open_frame_subscriber(context)
    target_publisher = open_target_publisher(context)

    tracker = TargetTracker()
    timer = TimerStream()
    timer.add_stream("inference")
    print("Starting!")

    while True:
        frame = decode_frame(frame_subscriber.recv())
        if frame is None:
            continue

        timer.set_baseline("inference")
        boxes = model_inference(frame, torch_device, image_transforms, model, threshold)
        timer.add_time("inference")
        timer.format_streamPrint("inference")

        detected_centroid = box_centroid_in_frame(boxes[0], frame.shape) if len(boxes) else None
        centroid = tracker.update(detected_centroid)
        publish_centroid(target_publisher, centroid)

        if show:
            show_frame(frame, centroid)


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("-w", "--weights", default=DEFAULT_WEIGHTS_PATH, help="path to load weights from")
    parser.add_argument("-t", "--threshold", type=float, default=0.65, help="minimum box score to accept")
    parser.add_argument("-s", "--show", action="store_true", help="display frames with cv2")
    return parser.parse_args()


if __name__ == "__main__":
    arguments = parse_arguments()
    run(arguments.weights, arguments.threshold, arguments.show)
