import asyncio
import websockets
import cv2
import websockets.exceptions

async def send_frames(uri):
    try:
        async with websockets.connect(uri) as websocket:
            cap = cv2.VideoCapture(0)  # Open webcam

            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    break

                _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                await websocket.send(buffer.tobytes())  # Send frame as binary

                # Check for a key press to exit (if running in an environment where cv2.waitKey works)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Shutting down client connection.")
                    await websocket.close()  # Gracefully close the connection
                    break

            cap.release()

    except websockets.exceptions.ConnectionClosedError:
        print("Server closed the connection.")
    
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == '__main__':
    asyncio.get_event_loop().run_until_complete(send_frames('ws://10.0.0.231:8080'))