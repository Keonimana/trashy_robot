#!/usr/bin/env python3
import numpy as np
import math
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
from matplotlib import pyplot as plt
from timerStream import TimerStream 


#Visualization constants: 
rgbWeight = 0.4 
depthWeight = 0.6

rgbCamSocket = dai.CameraBoardSocket.CAM_A

FPS = 30 
baseline = 75 
fov = 71.86
width = 640
focal = width / (2*math.tan(fov / 2 / 180 * math.pi)) 

def configure_IMU(imu): 
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW],35)
    imu.setBatchReportThreshold(1) 
    imu.setMaxBatchReports(10)
def configure_RGB(camRgb): 
    camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setFps(FPS) 
def configure_Mono(monoLeft,monoRight): 
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setCamera("left")
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setCamera("right")
def configure_Stereo(stereo):
    stereo.setRectifyEdgeFillColor(0) 
    stereo.setLeftRightCheck(True)
    stereo.setExtendedDisparity(True) 
    stereo.setSubpixel(False)
    stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
    #Add a few more post-processing features potentially?
    stereo.initialConfig.setConfidenceThreshold(200) 
    stereo.setBaseline(baseline/10) 
    stereo.setFocalLength(focal) 
def configure_Sync(sync):
    sync.setSyncThreshold(timedelta(milliseconds=50))
    sync.setSyncAttempts(-1) 
def configure_Spatial(spatial): 
    spatial.setWaitForConfigInput(False) 
    config = dai.SpatialLocationCalculatorConfigData() 
    config.depthThresholds.lowerThreshold = 100 
    config.depthThresholds.upperThreshold = 10000
    calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
    config.calculationAlgorithm = calculationAlgorithm 
    topLeft = dai.Point2f(200,200) 
    bottomRight = dai.Point2f(400,400) 
    config.roi = dai.Rect(topLeft,bottomRight)
    spatial.initialConfig.addROI(config) 
    return config

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
        return (pt1,pt2) 

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

def process_Stereo(depth,config): 
    t.set_baseline("depth")
    maxDisp = config.getMaxDisparity() 
    dispIntegerLevels = maxDisp 
    dispScaleFactor = baseline * focal 
    with np.errstate(divide="ignore"): 
        depth = dispScaleFactor / depth 
    depth = (depth * 255. / dispIntegerLevels).astype(np.uint8) 
    depth = cv2.applyColorMap(depth, cv2.COLORMAP_HOT) 
   # print(np.min(depth),np.mean(depth), np.max(depth), depth.shape)
    depth_bgr = cv2.cvtColor(depth, cv2.COLOR_RGB2BGR)  # Convert to BGR
    cv2.imshow("depth", depth) 
    t.add_time("depth") 
    t.format_streamPrint("depth")

def model_inference(rgb_frame,image_transforms,model,threshold=0.65): 
    t.set_baseline("inference") 
    img = Image.fromarray(rgb_frame[:,:,[2,1,0]])  
    tensor = image_transforms(img) 
    output = model([tensor]) 
    scores = output[0]['scores'] 
    boxes = output[0]['boxes']  
    high_score_indices = scores > threshold
    t.add_time("inference") 
    t.format_streamPrint("inference")
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
    spatial = pipeline.create(dai.node.SpatialLocationCalculator) 

    #Configure Nodes
    configure_IMU(imu)
    configure_RGB(camRgb)
    configure_Mono(monoLeft,monoRight)
    configure_Stereo(stereo)
    stereo.setDepthAlign(rgbCamSocket) 
    configure_Sync(sync) 
    spatial_config = configure_Spatial(spatial) 




    #Xout Nodes
    xoutStereo = pipeline.create(dai.node.XLinkOut) 
    xoutStereo.setStreamName("stereo") 
    xoutLeft = pipeline.create(dai.node.XLinkOut) 
    xoutLeft.setStreamName("left") 
    xoutRight = pipeline.create(dai.node.XLinkOut) 
    xoutRight.setStreamName("right") 
    xoutSync = pipeline.create(dai.node.XLinkOut) 
    xoutSync.setStreamName("sync") 
#    xoutSpatial = pipeline.create(dai.node.XLinkOut)
#    xoutSpatial.setStreamName("spatial") 

    #Probably create an Xin that is also responsible for setting the STereo config 
    xinSpatialConfig = pipeline.create(dai.node.XLinkIn) 
    xinSpatialConfig.setStreamName("spatialConfig") 
    xinSpatialConfig.out.link(spatial.inputConfig) 
   # monoLeft.out.link(xoutLeft.input)
   # monoRight.out.link(xoutRight.input)

    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)
    stereo.disparity.link(xoutStereo.input)
    stereo.depth.link(spatial.inputDepth) 

    #stereo.disparity.link(sync.inputs["disparity"]) 
    #stereo.syncedLeft.link(sync.inputs["left"]) 
    #stereo.syncedRight.link(sync.inputs["right"]) 
    #stereo.confidenceMap.link(sync.inputs["conf"]) 
    #stereo.outConfig.link(sync.inputs["config"])
    #stereo.depth.link(sync.inputs["depth"]) 
    #stereo.disparity.link(sync.inputs["disparity"]) 

    #Syn IMU and RGB 
    camRgb.video.link(sync.inputs["rgb"]) 
    imu.out.link(sync.inputs["imu"]) 
    spatial.out.link(sync.inputs["spatial"]) #Lets just move this into the sync node
    #Xout Links
    sync.out.link(xoutSync.input)

    #PyTorch 
    model = torchvision.models.detection.ssdlite.ssdlite320_mobilenet_v3_large(weights_backbone = "DEFAULT", num_classes=2)
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
    newConfig = False 

    with dai.Device(pipeline) as device:
        print("Device Connected") 
        qSync = device.getOutputQueue(name ="sync", maxSize =1, blocking = False)  #Wonder if its not clearnig the q
        qStereo = device.getOutputQueue(name ="stereo", maxSize =1, blocking = False) 
        #qSpatial = device.getOutputQueue(name ="spatial", maxSize =1, blocking = False) 
        qSpatialConfig = device.getInputQueue(name ="spatialConfig", maxSize =1, blocking = False) 
        displayFrame = None
        frameRgb = None 
        frameDisp = None 
        bbox = None
        spatialBox = None
        spatialBoxConfig = None 
        t.add_stream("imu") 
        t.add_stream("inference") 
        t.add_stream("depth") 
        t.add_stream("sync") 
        while True:
            inSync = qSync.tryGet() 
            inStereo = qStereo.tryGet()
            if inStereo is not None: 
                t.add_time("depth") 
                frameDisp = inStereo.getCvFrame() 
                maxDisparity = stereo.initialConfig.getMaxDisparity() 
                frameDisp = (frameDisp * 255./maxDisparity).astype(np.uint8) 
                frameDisp = cv2.pyrDown(frameDisp) 
                t.format_streamPrint("depth") 
                t.set_baseline("depth") 
            if inSync is not None: 
                t.add_time("sync") 
                t.format_streamPrint("sync") 
                imuPackets = inSync["imu"].packets 
                rgbFrames = inSync["rgb"] 
                spatialData = inSync["spatial"] 
                #depth = inSync["depth"]
                #disparity = inSync["disparity"].getCvFrame()
                #conf_map = inSync["conf"].getCvFrame() 
                process_Imu(imuPackets)
                #process_Stereo(depth.getFrame(),stereo.initialConfig)
                #cv2.imshow("disparity", disparity) 
                frame = cv2.pyrDown(rgbFrames.getCvFrame())
                frameRgb = frame
                #depth = np.where(depth > 8000, 0, depth)
                #print(np.min(depth),np.mean(depth), np.max(depth), depth.shape)
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
                    box = display_bbox(frame,boxes,scores) 
                    if args['show']:
                        cv2.imshow("Display Window",frame)
                    if box is not None: 
                        bbox = box 
                        pt1 = bbox[0] 
                        pt2 = bbox[1] 
                        topLeft = dai.Point2f(pt1[0],pt1[1]) 
                        bottomRight = dai.Point2f(pt2[0],pt2[1]) 
                        spatial_config.roi = dai.Rect(topLeft,bottomRight) 
                        cfg = dai.SpatialLocationCalculatorConfig() 
                        cfg.addROI(spatial_config) 
                        qSpatialConfig.send(cfg) 
                        print("Sending Config") 
                        space_vec = spatialData.getSpatialLocations() 
                        for depthData in space_vec: 
                            spatialBox = (int(depthData.spatialCoordinates.x),int(depthData.spatialCoordinates.y),int(depthData.spatialCoordinates.z)) 
                            break
                counter += 1 
                t.set_baseline("sync") 
            if frameRgb is not None and frameDisp is not None and spatialBox is not None and bbox is not None: 
                if len(frameDisp.shape) < 3:
                    frameDisp = cv2.cvtColor(frameDisp, cv2.COLOR_GRAY2BGR)
                
                print(frameDisp.shape) 
                print(frameRgb.shape) 
                blended = cv2.addWeighted(frameRgb, rgbWeight, frameDisp, depthWeight, 0)
                pt1 = bbox[0] 
                pt2 = bbox[1] 
                print(f"SptialBox ({pt1},{pt2})") 
                fontType = cv2.FONT_HERSHEY_SIMPLEX
                cv2.rectangle(blended, pt1,pt2, (0,255,0),2)
                cv2.putText(blended, f"X: {spatialBox[0]} mm", (pt1[0] + 10, pt1[1] + 20), fontType, 0.5,(0,255,255))
                cv2.putText(blended, f"Y: {spatialBox[1]} mm", (pt1[0] + 10, pt1[1] + 35), fontType, 0.5, (0,255,255))
                cv2.putText(blended, f"Z: {spatialBox[2]} mm", (pt1[0] + 10, pt1[1] + 50), fontType, 0.5, (0,255,255))
                cv2.imshow("blend", blended)
                frameRgb = None
                frameDisp = None
                bbox = None
                spatialBox = None 

            if args['show']: 
                cv2.waitKey(1)



if __name__ == "__main__": 
    parser = argparse.ArgumentParser() 
    parser.add_argument('-w','--weights', default = 'weights/', help = 'path to save/load weights') 
    parser.add_argument('-s', '--show', action = 'store_true', help = 'Cv2 displays', default = True) 
    parser.add_argument('-t', '--threshold', default = 0.65, help = 'Model Threshold for box')
    args = vars(parser.parse_args()) 
    run(args) 


