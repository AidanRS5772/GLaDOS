import asyncio
import websockets
import cv2

async def send_frames(uri):
    async with websockets.connect(uri) as websocket:
        cap = cv2.VideoCapture(0)  # Open webcam

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            
            _, buffer = cv2.imencode('.jpg', frame)
            await websocket.send(buffer.tobytes())  # Send frame as binary

        cap.release()


if __name__ == '__main__':
    asyncio.get_event_loop().run_until_complete(send_frames('ws://10.0.0.231:8080'))