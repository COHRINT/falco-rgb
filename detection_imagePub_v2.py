# https://github.com/ultralytics/ultralytics.git
# python -m pip install --upgrade pip
# pip install opencv-python
# pip install opencv-contrib-python
# pip install ultralytics
# yolo predict model=yolov8n.pt source='https://ultralytics.com/images/bus.jpg'
import asyncio
import json
import cv2
from ultralytics import YOLO
from PIL import Image
import base64
import websockets
import winsound

## Make sure the yolo models are in the directory
#model = YOLO("yolov8n.pt")  #<- nano model
model = YOLO("yolov8s.pt")  #<- s model
#model = YOLO("yolov8x.pt")  #<- x model
#model = YOLO("yolov8m.pt")   #<- m model
#model = YOLO("yolov8n_best.pt")   #<- n  trained model
#model = YOLO("yolov8s_best.pt")   #<- s trained model
#model = YOLO("yolov8l_best.pt")   #<- l trained model
#model.info()

# Open the video file
#video_path = "rtsp://rinao:unicorn@192.168.0.100:8554/streaming/live/1" ## Testing with our drone
#video_path = "rtsp://210.99.70.120:1935/live/cctv002.stream" ## Testing with random South Korean rtsp stream
#video_path = "dji_drone.mp4" ## Testing with recorded dataset
#video_path = "orbit_rgb1.mp4" ## Testing with recorded dataset
#video_path = "level3_rgb.MP4" ## Testing with recorded dataset
#video_path = "straight_rgb.mp4" ## Testing with recorded datasets

#cam_feed = cv2.VideoCapture(video_path)
cam_feed = cv2.VideoCapture(0)
CONFIDENCE_THRESHOLD = 0.5

if not cam_feed.isOpened():
    print("Error: Could not open video stream")
    exit(0)
else:
    print("Video stream opened successfully")

def img_to_base64_string(image):
    _,image = cv2.imencode('.jpg',image)
    imgBase64 = base64.b64encode(image.tobytes())
    return f"data:image/jpeg;base64,{imgBase64.decode()}"

async def send_json(ws_client, data):
    json_data = json.dumps(data)
    await ws_client.send(json_data)
    print(f"Sent JSON data: {data['args']['event']}")

async def main():
    uri = "ws://192.168.0.101:8085" ## Need to use this when connecting to the server
    uri = "ws://localhost:8085" ## Testing locally
    count_frame = 0
    async with websockets.connect(uri) as ws_client:
        while cam_feed.isOpened():
            ret, frame = cam_feed.read()
            if not ret:
                print("Error: Could not read frame")
                break
            count_frame += 1
            if count_frame % 1 == 0: ## Change the modulo depending on the computing power/capacity. Needs work. Use at least 20 for heavier models
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame_pil = Image.fromarray(frame_rgb)

                # Perform inference on the frame
                results = model(frame_pil)

                # Correctly access boxes and names
                boxes = results[0].boxes
                names = results[0].names
                preds = []
                # Process the results
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())  # Extract coordinates and convert to integers
                    conf = box.conf[0].item()  # Confidence score

                    # Ghritachi - added new message -- send confidence score to hippo every time
                    await send_json(ws_client, {"action": "ConfidenceScore", "args": {"score": conf}})

                    if conf < CONFIDENCE_THRESHOLD: ## Set the correct threshold
                        continue
                    cls = int(box.cls[0].item())  # Class label index
                    label = f"{names[cls]} {conf:.2f}"  # Class name with confidence
                    preds.append({'name': names[cls], 'percentage_probability': conf})

                    # Draw bounding box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                new_size = (800, 600)
                resized_image = cv2.resize(frame, new_size)
                cv2.imshow("YOLO Output", resized_image)
                imageUrl = img_to_base64_string(resized_image)
                for detection_list in preds:
                    if detection_list['name'] == 'person':
                        await send_json(ws_client,{"action": "NewAlert", "args": {"event": "alert", "image": imageUrl}})
                        frequency = 1250  # Set Frequency To 2500 Hertz
                        duration = 1000  # Set Duration To 1000 ms == 1 second
                        winsound.Beep(frequency, duration)
                        #os.system("start welcome.mp3")
                        break

            # Press 'q' to quit the video stream
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


asyncio.run(main())
cam_feed.release()
cv2.destroyAllWindows()

