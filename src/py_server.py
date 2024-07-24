import numpy
import cv2
import socket
import threading

def handle_client(client_socket, addr):
    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        _, img_encoded = cv2.imencode('.jpg', frame)
        img_bytes = img_encoded.tobytes()

        client_socket.sendall(len(img_bytes).to_bytes(4, byteorder='big'))
        client_socket.sendall(img_bytes)

    cap.release()

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