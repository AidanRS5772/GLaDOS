import asyncio
import websockets
import cv2
import websockets.exceptions
import threading
import sys
import select
import platform

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

async def send_frames(uri, stop_event):
    try:
        async with websockets.connect(uri) as websocket:
            cap = cv2.VideoCapture(0)  # Open webcam

            while cap.isOpened():
                ret, frame = cap.read()
                if not ret or stop_event.is_set():
                    break

                _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                await websocket.send(buffer.tobytes())  # Send frame as binary

                if stop_event.is_set():
                    break

                ready_signal = await websocket.recv()
                if ready_signal != "Ready":
                    print("Unexpected message from server, closing connection.")
                    break

    except websockets.exceptions.ConnectionClosedError:
        print("Server closed the connection.")
        stop_event.set()  # Signal the stop event to shut down the client
    
    except websockets.exceptions.ConnectionClosedOK:
        print("Connection closed normally (1000). Exiting.")
        stop_event.set()  # Signal the stop event to shut down the client

    except Exception as e:
        print(f"An error occurred: {e}")
        stop_event.set()  # Signal the stop event to shut down the client
    
    finally:
        cap.release()
        sys.exit(0)  # Ensure the client exits after the connection is closed

async def main():
    stop_event = threading.Event()
    
    # Start the shutdown check in a separate thread
    input_thread = threading.Thread(target=check_for_shutdown, args=(stop_event,))
    input_thread.start()

    # Run the asyncio event loop to send frames
    await send_frames('ws://10.0.0.231:8080', stop_event)

    # Ensure the input thread completes
    input_thread.join()

    print("Client has shut down.")

if __name__ == '__main__':
    asyncio.run(main())


