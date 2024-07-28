import cv2
import socket
import threading

def handle_client(client_socket, addr):
    cap = cv2.VideoCapture(0)
    fourcc = cv2.VideoWriter_fourcc(*'H264')  # Use H.264 codec
    out = cv2.VideoWriter('pipe:', fourcc, 20.0, (640, 480))  # Pipe for FFmpeg

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        out.write(frame)  # Write frame to the pipe

        # Read encoded frame from the pipe
        encoded_frame = out.read()
        client_socket.sendall(len(encoded_frame).to_bytes(4, byteorder='big'))
        client_socket.sendall(encoded_frame)

    cap.release()
    out.release()

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
