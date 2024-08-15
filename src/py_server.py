import cv2
import socket
import threading

def handle_client(client_socket, addr):
    # Open the webcam
    cap = cv2.VideoCapture(0)
    
    # Set up VideoWriter to encode in H264
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = cv2.VideoWriter_fourcc(*'H264')
    out = cv2.VideoWriter('appsrc ! videoconvert ! video/x-raw,format=I420 ! x264enc ! video/x-h264,profile=baseline ! appsink', fourcc, 30, (width, height))

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Write the frame to the VideoWriter (encodes it)
        out.write(frame)
        
        # Capture the encoded frame
        encoded_frame = out.retrieve()
        
        # Send the frame length and then the frame itself
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