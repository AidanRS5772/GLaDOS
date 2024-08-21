import cv2
import socket
import struct

MAX_UDP_SIZE = 65507  # Maximum UDP packet size (65,507 bytes)

def serve():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(('0.0.0.0', 12345))

    print("Server is listening...")

    print("Waiting for the client to send a hello message...")
    message, client_address = server_socket.recvfrom(1024)
    print(f"Received hello message from {client_address}")

    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Encode the frame as a JPG image
        ret, jpg_data = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])

        if ret:
            jpg_data = jpg_data.tobytes()

            # Split the JPG data into chunks
            for i in range(0, len(jpg_data), MAX_UDP_SIZE):
                chunk = jpg_data[i:i + MAX_UDP_SIZE]
                # Send each chunk to the client
                server_socket.sendto(struct.pack("H", i) + chunk, client_address)

    cap.release()
    server_socket.close()

if __name__ == "__main__":
    serve()
