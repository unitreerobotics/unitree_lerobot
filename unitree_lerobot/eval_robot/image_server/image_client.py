"""
This file contains utilities for recording frames from cameras. For more info look at `OpenCVCamera` docstring.
"""

import struct
import threading
import time
from collections import deque
from multiprocessing import shared_memory

import cv2
import numpy as np
import zmq

from unitree_lerobot.eval_robot.image_server import zmq_msg

import logging_mp
logger_mp = logging_mp.get_logger(__name__, level=logging_mp.INFO)

class ImageClient:
    def __init__(self, host="192.168.123.164", request_port=60000):
        """
        Args:
            server_address:   IP address of image host server
            requset_port:     Port for request camera configuration
        """
        self._host = host
        self._request_port = request_port

        # subscriber and requester setup
        self._subscriber_manager = zmq_msg.SubscriberManager.get_instance()
        self._requester  = zmq_msg.Requester(self._host, self._request_port)
        self._cam_config = self._requester.request()

        if self._cam_config is None:
            raise RuntimeError("Failed to get camera configuration.")
        
        if self._cam_config['head_camera']['enable_zmq']:
            self._subscriber_manager.subscribe(self._host, self._cam_config['head_camera']['zmq_port'])

        if self._cam_config['left_wrist_camera']['enable_zmq']:
            self._subscriber_manager.subscribe(self._host, self._cam_config['left_wrist_camera']['zmq_port'])

        if self._cam_config['right_wrist_camera']['enable_zmq']:
            self._subscriber_manager.subscribe(self._host, self._cam_config['right_wrist_camera']['zmq_port'])

        if not self._cam_config['head_camera']['enable_zmq'] and not self._cam_config['head_camera']['enable_webrtc']:
            logger_mp.warning("[Image Client] NOTICE! Head camera is not enabled on both ZMQ and WebRTC.")

    # --------------------------------------------------------
    # public api
    # --------------------------------------------------------
    def get_cam_config(self):
        return self._cam_config

    def get_head_frame(self):
        return self._subscriber_manager.subscribe(self._host, self._cam_config['head_camera']['zmq_port'])
    
    def get_left_wrist_frame(self):
        return self._subscriber_manager.subscribe(self._host, self._cam_config['left_wrist_camera']['zmq_port'])
    
    def get_right_wrist_frame(self):
        return self._subscriber_manager.subscribe(self._host, self._cam_config['right_wrist_camera']['zmq_port'])
        
    def close(self):
        self._subscriber_manager.close()
        logger_mp.info("Image client has been closed.")

def main():
    # command line args
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', type=str, default='192.168.123.164', help='IP address of image server')
    args = parser.parse_args()

    # Example usage with three camera streams
    client = ImageClient(host=args.host)
    cam_config = client.get_cam_config()

    running = True
    while running:
        if cam_config['head_camera']['enable_zmq']:
            head_img, head_fps = client.get_head_frame()
            if head_img is not None:
                logger_mp.info(f"Head Camera FPS: {head_fps:.2f}")
                logger_mp.debug(f"Head Camera Shape: {cam_config['head_camera']['image_shape']}")
                logger_mp.debug(f"Head Camera Binocular: {cam_config['head_camera']['binocular']}")
                cv2.imshow("Head Camera", head_img)

        if cam_config['left_wrist_camera']['enable_zmq']:
            left_wrist_img, left_wrist_fps = client.get_left_wrist_frame()
            if left_wrist_img is not None:
                logger_mp.info(f"Left Wrist Camera FPS: {left_wrist_fps:.2f}")
                logger_mp.debug(f"Left Wrist Camera Shape: {cam_config['left_wrist_camera']['image_shape']}")
                cv2.imshow("Left Wrist Camera", left_wrist_img)

        if cam_config['right_wrist_camera']['enable_zmq']:
            right_wrist_img, right_wrist_fps = client.get_right_wrist_frame()
            if right_wrist_img is not None:
                logger_mp.info(f"Right Wrist Camera FPS: {right_wrist_fps:.2f}")
                logger_mp.debug(f"Right Wrist Camera Shape: {cam_config['right_wrist_camera']['image_shape']}")
                cv2.imshow("Right Wrist Camera", right_wrist_img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            logger_mp.info("Exiting image client on user request.")
            running = False
            # clean up
            client.close()
            cv2.destroyAllWindows()
        # Small delay to prevent excessive CPU usage
        time.sleep(0.002)

if __name__ == "__main__":
    main()
