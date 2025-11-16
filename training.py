import cv2

def main():
    # Open the default camera (0)
    cap = cv2.VideoCapture(1)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # Set optional parameters (e.g. resolution)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FOCUS, 0)
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        # Display the frame
        cv2.imshow("Camera Stream", frame)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
