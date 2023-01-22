#!/usr/bin/env python3
import cv2
import numpy as np
from time import sleep

CAMERA_MODES = [
    (0, 3264, 2464, 21, "full"),
    (1, 3264, 1848, 28, "partial"),
    (2, 1920, 1080, 30, "partial"),
    (3, 1640, 1232, 30, "full"),
    (4, 1280, 720, 60, "partial"),
    (5, 1280, 720, 120, "partial"),
]
EXPOSURE_TIMERANGES = {"sports": [
    100000, 80000000], "night": [100000, 1000000000]}


def gst_pipeline_string(mode, exposure='sports'):
    # Parameters from the camera_node
    # https://github.com/duckietown/dt-duckiebot-interface/blob/3d11f3d77e2754f2a3be73e93f27c43a90eb21ec/packages/camera_driver/src/jetson_nano_camera_node.py#L178

    camera_mode, res_w, res_h, fps, _ = CAMERA_MODES[mode]
    # compile gst pipeline
    gst_pipeline = """ \
            varguscamerasrc \
            sensor-mode={} exposuretimerange="{} {}" ! \
            video/x-raw(memory:NVMM), \
            width={}, height={}, format=NV12, framerate={}/1 ! \
            nvvidconv ! 
            format=BGRx ! \
            videoconvert ! \
            video/x-raw, format=BGR ! appsink
        """.format(
        camera_mode,
        *EXPOSURE_TIMERANGES[exposure],
        res_w,
        res_h,
        fps
    )

    print("Using GST pipeline: ``".format(gst_pipeline))
    return gst_pipeline


cap = cv2.VideoCapture()
cap.open(gst_pipeline_string(mode=3), cv2.CAP_GSTREAMER)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # define the list of boundaries
    boundaries = [
        ([17, 15, 100], [50, 56, 200]),
        ([86, 31, 4], [220, 88, 50]),
        ([25, 146, 190], [62, 174, 250]),
        ([103, 86, 65], [145, 133, 128])
    ]

    # loop over the boundaries
    for (lower, upper) in boundaries:
        # create NumPy arrays from the boundaries
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")
        # find the colors within the specified boundaries and apply
        # the mask
        mask = cv2.inRange(frame, lower, upper)
        output = cv2.bitwise_and(frame, frame, mask=mask)

        # show the images
        cv2.imshow("images", np.hstack([frame, output]))
        cv2.waitKey(0)

    sleep(1)

