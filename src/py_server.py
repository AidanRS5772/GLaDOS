import cv2
import socket
import struct

def serve():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(('0.0.0.0', 12345))

    print("Server is listening...")

    # Wait for the client to send the first message
    print("Waiting for the client to send a hello message...")
    message, client_address = server_socket.recvfrom(1024)  # Buffer size is 1024 bytes
    print(f"Received hello message from {client_address}")

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

            # Send the message to the client's address
            server_socket.sendto(msg, client_address)

    cap.release()
    server_socket.close()

if __name__ == "__main__":
    serve()
