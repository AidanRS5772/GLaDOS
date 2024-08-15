import cv2

# Initialize video capture object
cap = cv2.VideoCapture(0)  # 0 for default webcam

# Set desired frame width and height
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'avc1')  # H264 codec
out = cv2.VideoWriter('output.mp4', fourcc, 30.0, (640, 480))

while True:
    ret, frame = cap.read()  # Read a frame from the webcam

    if not ret:
        print("Error reading frame")
        break

    # Write the frame to the output video file
    out.write(frame)

    # Display the frame
    cv2.imshow('Webcam', frame)

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
out.release()
cv2.destroyAllWindows()