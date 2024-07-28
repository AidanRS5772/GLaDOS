import cv2
import socket
import threading

def handle_client(client_socket, addr):
    # Open video capture
    cap = cv2.VideoCapture(0)

    # Create a VideoWriter object with H.264 codec
    fourcc = cv2.VideoWriter_fourcc(*'H264')  # or use 'X264' or other codec if available
    width, height = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    video_writer = cv2.VideoWriter('stream.mp4', fourcc, fps, (width, height))

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Write the frame to the video file (H.264 encoded)
        video_writer.write(frame)

        # Read the encoded frame
        with open('stream.mp4', 'rb') as f:
            video_bytes = f.read()

        client_socket.sendall(len(video_bytes).to_bytes(4, byteorder='big'))
        client_socket.sendall(video_bytes)

    cap.release()
    video_writer.release()
    print(f"Video has been streamed to {addr}")
    client_socket.sendall(b"END_STREAM")
    client_socket.close()


def serve():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', 12345))
    server_socket.listen(1)

    print("Server is listening...")

    while True:
        client_socket, addr = server_socket.accept()
        print(f"Connection from {addr} has been established.")
        client_handler = threading.Thread(target=handle_client, args=(client_socket, addr))
        client_handler.start()


if __name__ == "__main__":
    serve()