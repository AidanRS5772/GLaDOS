import cv2
import socket
import struct

def handle_client(client_socket):
    client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)  # Receive buffer size
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4096)  # Send buffer size

    # Open video capture from the webcam
    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Encode the frame as a JPG image
        ret, jpg_data = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 100])

        if ret:
            # Serialize the length of the encoded JPG and the image data
            msg = struct.pack("Q", len(jpg_data)) + jpg_data.tobytes()
            client_socket.sendall(msg)

    cap.release()
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
        handle_client(client_socket, addr)

if __name__ == "__main__":
    serve()

