import asyncio
import websockets
import cv2
import websockets.exceptions
import threading
import sys
import select
import platform
from libcamera import picamera2

if platform.system() == "Windows":
    import msvcrt

def check_for_shutdown(stop_event):
    while not stop_event.is_set():
        if platform.system() == "Windows":
            if msvcrt.kbhit():
                command = msvcrt.getch().decode('utf-8')
                if command.lower() == 'q':
                    print("Shutting down client connection.")
                    stop_event.set()
                    break
        else:
            if sys.stdin in select.select([sys.stdin], [], [], 1)[0]:
                command = input().strip()
                if command.lower() == 'q':
                    print("Shutting down client connection.")
                    stop_event.set()
                    break


JPG_IMAGE_QUALITY = 90

async def send_frames(uri, stop_event):
    try:
        async with websockets.connect(uri) as websocket:
            picam2 = Picamera2()
            config = picam2.create_preview_configuration(main={"size": (2592, 1944)})
            picam2.configure(config)
            picam2.start()

            while True:
                frame = picam2.capture_array()
                if stop_event.is_set():
                    break

                _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPG_IMAGE_QUALITY])
                await websocket.send(buffer.tobytes())  
                if stop_event.is_set():
                    break

                ready_signal = await websocket.recv()
                if ready_signal != "ACK":
                    print(f"Unexpected message from server : {ready_signal}")
                    break

    except websockets.exceptions.ConnectionClosedError:
        print("Server closed the connection.")
        stop_event.set()
    
    except websockets.exceptions.ConnectionClosedOK:
        print("Connection closed normally (1000). Exiting.")
        stop_event.set()

    except Exception as e:
        print(f"An error occurred: {e}")
        stop_event.set()
    
    finally:
        cap.release()
        sys.exit(0)

async def main():
    stop_event = threading.Event()
    
    input_thread = threading.Thread(target=check_for_shutdown, args=(stop_event,))
    input_thread.start()

    await send_frames('ws://10.0.0.231:8080', stop_event)

    input_thread.join()

    print("Client has shut down.")

if __name__ == '__main__':
    asyncio.run(main())


