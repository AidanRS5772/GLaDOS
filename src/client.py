import asyncio
import websockets
import cv2
import sys
import platform
import numpy as np
import struct  # For packing data into bytes

# If running on Windows, import msvcrt
if platform.system() == "Windows":
    import msvcrt

# Constants for motion detection
PRE_THRESH = 400     # Threshold for KNN background subtractor
FRAME_HIST = 120     # Length of frame history for KNN background subtractor
L_KERNAL_SZ = 7      # Size of convolution kernel for large morphologies
S_KERNAL_SZ = 3      # Size of convolution kernel for small morphologies
POST_THRESH = 50     # Grayscale limit for post-thresholding of mask
AREA_THRESH = 5000   # Minimum threshold for identifying object

# Initialize the background subtractor (KNN with parameters)
KNN = cv2.createBackgroundSubtractorKNN(history=FRAME_HIST, dist2Threshold=PRE_THRESH)

# Define kernels for morphological operations
kernel_S = np.ones((S_KERNAL_SZ, S_KERNAL_SZ), np.uint8)
kernel_L = np.ones((L_KERNAL_SZ, L_KERNAL_SZ), np.uint8)

# Shared variables
shared_frame = None
frame_lock = asyncio.Lock()
frame_available_event = asyncio.Event()

stop_event = asyncio.Event()

def find_motion(frame):
    fg_mask = KNN.apply(frame)
    _, clean_fg_mask = cv2.threshold(fg_mask, POST_THRESH, 255, cv2.THRESH_BINARY)

    clean_fg_mask = cv2.morphologyEx(clean_fg_mask, cv2.MORPH_OPEN, kernel_S)
    clean_fg_mask = cv2.morphologyEx(clean_fg_mask, cv2.MORPH_CLOSE, kernel_L)

    contours, _ = cv2.findContours(
        clean_fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    max_area = 0
    max_bounding_rect = None

    if contours:
        max_bounding_rect = max(
            contours, key=lambda c: cv2.boundingRect(c)[2] * cv2.boundingRect(c)[3]
        )
        x, y, w, h = cv2.boundingRect(max_bounding_rect)
        max_area = w * h

        if max_area > AREA_THRESH:
            center_x = x + w // 2
            center_y = y + h // 2
            return (center_x, center_y), (x, y, w, h)  # Return center and bounding box

    return None, None

async def check_for_shutdown():
    print("Press 'q' and Enter to shut down the client.")
    while not stop_event.is_set():
        if platform.system() == "Windows":
            if msvcrt.kbhit():
                key = msvcrt.getch()
                if key.decode('utf-8').lower() == 'q':
                    print("Shutting down client connection.")
                    stop_event.set()
        else:
            # Read from stdin asynchronously
            try:
                loop = asyncio.get_running_loop()
                future = loop.run_in_executor(None, sys.stdin.readline)
                result = await asyncio.wait_for(future, timeout=1.0)
                if result.strip().lower() == 'q':
                    print("Shutting down client connection.")
                    stop_event.set()
            except asyncio.TimeoutError:
                pass  # No input, continue

async def motion_detection():
    try:
        # Initialize camera
        from picamera2 import Picamera2
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"size": (640, 480)})
        picam2.configure(config)
        picam2.start()

        # Connect to coordinates WebSocket
        async with websockets.connect("ws://10.0.0.231:8081") as websocket_coords:
            while not stop_event.is_set():
                frame = picam2.capture_array()
                center_coords, _ = find_motion(frame)

                if center_coords:
                    # Update shared frame
                    async with frame_lock:
                        global shared_frame
                        shared_frame = frame.copy()  # Make a copy to avoid mutation
                        frame_available_event.set()  # Signal that a new frame is available

                    # Pack center coordinates into bytes (big-endian network order)
                    coords_packed = struct.pack('!ii', *center_coords)
                    await websocket_coords.send(coords_packed)

                await asyncio.sleep(0.01)  # Adjust as needed

    except Exception as e:
        import traceback
        print(f"An error occurred in motion_detection: {e}")
        traceback.print_exc()
        stop_event.set()

async def send_frames():
    global shared_frame
    try:
        # Connect to frames WebSocket
        async with websockets.connect("ws://10.0.0.231:8080") as websocket_frames:
            while not stop_event.is_set():
                # Wait for ACK from server
                ack_message = await websocket_frames.recv()
                if ack_message != "ACK":
                    print(f"Unexpected message from server: {ack_message}")
                    continue

                # Wait for a new frame to be available
                await frame_available_event.wait()

                # Get the latest frame
                async with frame_lock:
                    if shared_frame is not None:
                        frame_to_send = shared_frame.copy()
                        shared_frame = None  # Clear the shared frame
                        frame_available_event.clear()
                    else:
                        # No frame to send
                        continue

                # Encode frame to JPEG
                frame_to_send = cv2.cvtColor(frame_to_send, cv2.COLOR_BGR2RGB)

                # Set JPEG compression parameters
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]  # Adjust quality as needed

                # Encode frame to JPEG with compression parameters
                ret, buffer = cv2.imencode('.jpg', frame_to_send, encode_param)
                if not ret:
                    print("Failed to encode frame")
                    continue
                frame_data = buffer.tobytes()

                # Send the frame data
                await websocket_frames.send(frame_data)
                print("Frame sent")

    except Exception as e:
        import traceback
        print(f"An error occurred in send_frames: {e}")
        traceback.print_exc()
        stop_event.set()
async def main():
    await asyncio.gather(
        check_for_shutdown(),
        motion_detection(),
        send_frames(),
    )

if __name__ == "__main__":
    asyncio.run(main())
