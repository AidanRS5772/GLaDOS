import asyncio
import websockets
import cv2
import websockets.exceptions

async def send_frames(uri):
    async with websockets.connect(uri) as websocket:
        cap = cv2.VideoCapture(0)  # Open webcam

        async def check_for_shutdown():
            # This will run in a background thread to avoid blocking the main loop
            while True:
                command = await asyncio.get_running_loop().run_in_executor(None, input, "Type 'q' to quit: ")
                if command.lower() == 'q':
                    print("Shutting down client connection.")
                    await websocket.close()  # Gracefully close the connection
                    cap.release()  # Release the webcam
                    break

        # Start the shutdown check in the background
        shutdown_task = asyncio.create_task(check_for_shutdown())

        try:
            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    break

                _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                await websocket.send(buffer.tobytes())  # Send frame as binary

                # Check if the shutdown task is complete
                if shutdown_task.done():
                    break

        except websockets.exceptions.ConnectionClosedError:
            print("Server closed the connection.")
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            cap.release()
            if not shutdown_task.done():
                shutdown_task.cancel()  # Cancel the shutdown task if it hasn't finished

if __name__ == '__main__':
    asyncio.get_event_loop().run_until_complete(send_frames('ws://10.0.0.231:8080'))