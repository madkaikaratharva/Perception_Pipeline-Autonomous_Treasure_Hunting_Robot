#!/usr/bin/env python3

import os
import torch
from ultralytics import YOLO

# Get the current directory path
current_dir = os.path.dirname(os.path.abspath(__file__))
weights_folder = os.path.join(current_dir, 'models')

# Set device to CUDA
torch.cuda.set_device(0)

class ObjectDetector:
    def __init__(self,
                 model_name='yolov8m.pt'):
        
        # Initialize model params
        self.model_name = model_name
        self.weights_path = os.path.join(weights_folder, self.model_name)

        # Initialize the model
        self.model = YOLO(self.weights_path)
        self.model.to('cuda:0')

        # Get class names detected by the model
        self.detected_classes = self.model.names

    def get_detections(self, color_frame):

        # Get detections
        results = self.model(color_frame, stream=True, verbose=False, conf=0.65)

        # Convert torch arrays to list
        detections = [[int(x1), int(y1), int(x2), int(y2), score, int(class_id)]
                        for result in results
                        for x1, y1, x2, y2, score, class_id in result.boxes.data.tolist()]
            
        return detections