#!/usr/bin/env python3
import argparse 
import time
from pathlib import Path
import cv2
import depthai as dai
import sys 
import torchvision 
import torch 
import torchvision.transforms as transforms 
from PIL import Image 
from datetime import timedelta
import time
from timerStream import TimerStream 

def configure_IMU(imu): 
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW],35)
    imu.setBatchReportThreshold(1) 
    imu.setMaxBatchReports(10)
def configure_RGB(camRgb): 
    camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
def configure_Mono(monoLeft,monoRight): 
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setCamera("left")
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setCamera("right")
def configure_Stereo(stereo):
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
    stereo.setRectifyEdgeFillColor(0) 
    stereo.setLeftRightCheck(True)
    stereo.setExtendedDisparity(True) 
    stereo.setSubpixel(True)
def configure_Sync(sync):
    sync.setSyncThreshold(timedelta(milliseconds=50))
    sync.setSyncAttempts(-1) 

#Since we're sub-sampling the frames, we should continue drawing on the prev bounding box
def display_bbox(img,boxes,scores):
    if boxes is None or scores is None: 
        return
    for bbox,score in zip(boxes,scores): 
        xmin = bbox[0].item() / 320.0 * 960.0
        ymin = bbox[1].item() / 320.0 * 540.0
        xmax = bbox[2].item() / 320.0 * 960.0 
        ymax = bbox[3].item() / 320.0 * 540.0
        #Need to resize these poitns 
        pt1 = (int(xmin),int(ymin))
        pt2 = (int(xmax),int(ymax))
        cv2.rectangle(img, pt1, pt2, (0,255,0),2)
        cv2.putText(img,str(score.item()),pt1,cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12),2)

def timeDeltaToMilliS(delta) -> float:
    return delta.total_seconds()*1000

def process_Imu(imuPackets): 
    t.set_baseline("imu") 
    for p in imuPackets: 
        acceleroValues = p.acceleroMeter
        acceleroTs = acceleroValues.getTimestampDevice()
        imuF = "{:.06f}"
        tsF  = "{:.03f}"
        #print(f"Accelerometer [m/s^2]: x: {imuF.format(acceleroValues.x)} y: {imuF.format(acceleroValues.y)} z: {imuF.format(acceleroValues.z)}")
    t.add_time("imu") 

def process_Stereo(depth): 
    t.set_baseline("depth") 
    depth_map = depth.getCvFrame() 
    cv2.imshow("depth", depth_map) 
    t.add_time("depth") 

def model_inference(rgb_frame,image_transforms,model,threshold=0.65): 
    t.set_baseline("inference") 
    img = Image.fromarray(rgb_frame[:,:,[2,1,0]])  
    tensor = image_transforms(img) 
    output = model([tensor]) 
    scores = output[0]['scores'] 
    boxes = output[0]['boxes']  
    high_score_indices = scores > threshold
    t.add_time("inference") 
    return (boxes[high_score_indices],scores[high_score_indices]) 

def run(args): 
    clear_box_threshold = 10 
    no_box = 0 
    counter = 1
    global t
    pipeline = dai.Pipeline()

    #Create Nodes
    imu = pipeline.create(dai.node.IMU)
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    camRgb = pipeline.create(dai.node.ColorCamera)
    stereo = pipeline.create(dai.node.StereoDepth) 
    sync = pipeline.create(dai.node.Sync) 

    #Configure Nodes
    configure_IMU(imu)
    configure_RGB(camRgb)
    configure_Mono(monoLeft,monoRight)
    configure_Stereo(stereo)
    configure_Sync(sync) 

    #Xout Nodes
    xoutStereo = pipeline.create(dai.node.XLinkOut) 
    xoutStereo.setStreamName("stereo") 
    xoutSync = pipeline.create(dai.node.XLinkOut) 
    xoutSync.setStreamName("sync") 

    #Setup Links
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)
    #stereo.disparity.link(sync.inputs["disparity"]) 
    #stereo.syncedLeft.link(sync.inputs["left"]) 
    #stereo.syncedRight.link(sync.inputs["right"]) 
    stereo.depth.link(sync.inputs["depth"]) 
    camRgb.video.link(sync.inputs["rgb"]) 
    imu.out.link(sync.inputs["imu"]) 
    #Xout Links
    sync.out.link(xoutSync.input)
    #Load pytorch model 
    model = torchvision.models.detection.ssdlite.ssdlite320_mobilenet_v3_large(weights_backbone = "DEFAULT", num_classes=2)
    #weights0.05_64_30_27
    model.load_state_dict(torch.load(args['weights'])) 
    model.eval() 

    image_transforms = transforms.Compose([
        transforms.ColorJitter(brightness = (0.5,1.5), contrast = (0.5,1.5), saturation = (0.5,1.5), hue = (-0.1, 0.1)),
        transforms.Resize((320,320)),
        transforms.ToTensor(),  # Convert the image to a PyTorch tensor
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])

    boxes = None 
    scores = None
    #Set up timer streams 
    t = TimerStream() 

    with dai.Device(pipeline) as device:
        # Output queue will be used to get the rgb frames from the output defined above
        qSync = device.getOutputQueue(name ="sync", maxSize =30, blocking = False) 
        t.add_stream("imu") 
        t.add_stream("inference") 
        t.add_stream("depth") 
        t.add_stream("sync") 
        while True:
            inSync = qSync.tryGet() 
            if inSync is not None: 
                t.add_time("sync") 
                #Grab packets
                imuPackets = inSync["imu"].packets 
                rgbFrames = inSync["rgb"] 
                depth = inSync["depth"] 
                process_Imu(imuPackets)
                process_Stereo(depth)
                frame = cv2.pyrDown(rgbFrames.getCvFrame())
                if counter % 2 == 0:
                    curr_boxes, curr_scores = model_inference(frame,image_transforms,model,threshold=float(args['threshold']))
                    #some kind of jank logic to figure out if a true detection exists? 
                    if len(curr_boxes) >= 1: 
                        boxes = curr_boxes 
                        scores = curr_scores
                        no_box = 0 
                    else:
                        no_box += 1
                    if no_box > clear_box_threshold: 
                        boxes = None 
                        scores = None
                    display_bbox(frame,boxes,scores) 
                    if args['show']: 
                        cv2.imshow("Display Window",frame) 
                counter += 1 
                #Record the next time we get a sync packet 
                t.set_baseline("sync") 
            if args['show']: 
                cv2.waitKey(1)


if __name__ == "__main__": 
    parser = argparse.ArgumentParser() 
    parser.add_argument('-w','--weights', default = 'weights/', help = 'path to save/load weights') 
    parser.add_argument('-s', '--show', action = 'store_true', help = 'Cv2 displays', default = True) 
    parser.add_argument('-t', '--threshold', default = 0.65, help = 'Model Threshold for box')
    args = vars(parser.parse_args()) 
    run(args) 


