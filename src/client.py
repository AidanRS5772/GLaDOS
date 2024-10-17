import asyncio
import numpy as np
import websockets
import struct
from picamera2 import Picamera2
import cv2

PRE_THRESH = 400
FRAME_HIST = 120
L_KERNAL_SZ = 7
S_KERNAL_SZ = 3
POST_THRESH = 50
AREA_THRESH = 5000

KNN = cv2.createBackgroundSubtractorKNN(history = FRAME_HIST, dist2Threshold = PRE_THRESH)
S_KERNAL = np.ones((S_KERNAL_SZ, S_KERNAL_SZ), np.uint8)
L_KERNAL = np.ones((L_KERNAL_SZ, L_KERNAL_SZ), np.uint8)

def find_motion(frame):
    fg_mask = KNN.apply(frame)
    _, clean_fg_mask = cv2.threshold(fg_mask, POST_THRESH, 255, cv2.THRESH_BINARY)

    clean_fg_mask = cv2.morphologyEx(clean_fg_mask, cv2.MORPH_OPEN, S_KERNAL)
    clean_fg_mask = cv2.morphologyEx(clean_fg_mask, cv2.MORPH_CLOSE, L_KERNAL)

    contours, _ = cv2.findContours(clean_fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        max_countour = max(contours , key = lambda c: cv2.boundingRect(c)[2] * cv2.boundingRect(c)[3])
        x , y, w, h = cv2.boundingRect(max_countour)
        max_area = w * h

        if max_area > AREA_THRESH:
            c_x = x + w // 2
            c_y = y + h // 2
            return (c_x, c_y)

    return None

async def send_message(websocket, tag, data):
    await websocket.send(tag.encode() + data)

async def send_cord(ws, x, y):
    tag = "CORD".ljust(4, ' ')
    cord_data = struct.pack("!ii", x, y)
    await send_data(ws, tag, cord_data)
    print("Sent Cord")

async def main():
    async with websockets.connect("ws://10.0.0.232:8080") as ws:
        cam = Picamera2()
        cam.configure(cam.create_still_configuration())
        cam.start()

        send_cord = True

        while True:
            if send_cord:
                frame = cam.capture_array()
                motion_cord = find_motion(frame)
                if motion_cord:
                    x , y = motion_cord
                    await send_cord(ws, x, y)
                    send_cord = False
            
            server_signal = await ws.recv()
            if server_signal == "CORD":
                send_cord = True
            else:
                continue


# Run the main function
asyncio.get_event_loop().run_until_complete(main())