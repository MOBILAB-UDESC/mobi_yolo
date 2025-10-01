from cv_bridge import CvBridge
import cv2
# from inference import get_model
import numpy as np
import pyrealsense2 as rs
import supervision as sv
from ultralytics import YOLO


def camera_info(pipeline, config):
    """ Camera info debug function """
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    print(f'pipeline_wrapper: {pipeline_wrapper}')
    pipeline_profile = config.resolve(pipeline_wrapper)
    print(f'pipeline_profile: {pipeline_profile}')
    device = pipeline_profile.get_device()
    print(f'device: {device}')
    device_product_line = str(device.get_info(rs.camera_info.product_line))
    print(f'device_product_line: {device_product_line}')


def estimate_position(depth_image, bbox):
    """ Estimate the 3D position of the object in the bounding box """
    # Intrinsic parameters for the RealSense D435 camera at 1280x720 resolution
    fx = 644.219543457031
    fy = 644.219543457031
    ppx = 638.51171875
    ppy = 358.90625

    # Get the depth value at the center of the bounding box
    x_center = int((bbox[0] + bbox[2]) / 2)
    y_center = int((bbox[1] + bbox[3]) / 2)
    depth = depth_image[y_center, x_center]

    # Get xy position in mm
    x = depth * (x_center - ppx) / fx
    y = depth * (y_center - ppy) / fy

    print(f'Estimated depth at center of bbox: {depth}')
    return depth


# Camera initialization
print('Starting RealSense pipeline...')
debug = True
config = rs.config()
pipeline = rs.pipeline()

if debug:
    camera_info(pipeline, config)

# Configure both color and depth streams to have the same resolution
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# Detection config
bounding_box_annotator = sv.BoxAnnotator()
model = YOLO('mobi_yolo/yolo_detection/weights/best.pt')
# model = get_model(model_id="taylor-swift-records/3")

# OpenCV config
cv_bridge = CvBridge()

while True:
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    print(type(color_image))
    depth_color_image = cv2.applyColorMap(
        cv2.convertScaleAbs(
            depth_image,
            alpha=0.5),
        cv2.COLORMAP_JET)

    results = model.predict(color_image, conf=0.5, save=False, imgsz=(480, 640))[0]
    detections = sv.Detections.from_ultralytics(results)
    # results = model.infer(color_image)[0]
    # print(f'results: {results}')
    # detections = sv.Detections.from_inference(results)

    for bbox in detections.xyxy:
        depth = estimate_position(depth_image, bbox)
        print(f'Depth for bbox {bbox}: {depth} mm')

    color_image = bounding_box_annotator.annotate(scene=color_image, detections=detections)

    if debug:
        print(f'color_image.shape: {color_image.shape}')
        print(f'depth_image.shape: {depth_image.shape}')

    # color_image = cv2.rotate(color_image, cv2.ROTATE_90_CLOCKWISE)
    # color_image = cv2.rotate(color_image, cv2.ROTATE_90_CLOCKWISE)

    cv2.imshow('rgb', color_image)
    cv2.imshow('depth', depth_color_image)

    if cv2.waitKey(1) == 113:  # q key
        print('Stopping RealSense pipeline...')
        pipeline.stop()
        print('Pipeline stopped.')
        print('Destroying all windows...')
        cv2.destroyAllWindows()
        print('All windows destroyed. Exiting program.')
        break
