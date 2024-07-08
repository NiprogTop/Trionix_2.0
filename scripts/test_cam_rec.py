import cv2

stream_url = "http://192.168.1.100:8090/?action=stream"
cap = cv2.VideoCapture(stream_url)

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = None
file_counter = 1
frame_counter = 0
frame_rate = int(cap.get(cv2.CAP_PROP_FPS))
seconds_per_file = 300  # 5 minutes 30 fps

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    if out is None or frame_counter == frame_rate * seconds_per_file:
        if out is not None:
            out.release()
        out = cv2.VideoWriter(f"output_{file_counter}.avi", fourcc, frame_rate, (frame.shape[1], frame.shape[0]))
        file_counter += 1
        frame_counter = 0

    out.write(frame)
    frame_counter += 1

cap.release()
if out is not None:
    out.release()
cv2.destroyAllWindows()
