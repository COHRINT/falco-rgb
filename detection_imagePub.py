import asyncio
import cv2
import websockets
import json
import os
from julia.api import Julia
from imageai.Detection import VideoObjectDetection


jl = Julia(compiled_modules=False)
jl.eval('include("falco_function.jl")')

# Initialize belief using Julia's initialize_belief()
belief = jl.eval("reset_belief()")

execution_path = os.getcwd()
detector = VideoObjectDetection()
detector.setModelTypeAsYOLOv3()
detector.setModelPath(os.path.join(execution_path, "yolov3.pt"))
detector.loadModel()

cam_feed = cv2.VideoCapture(0)
#cam_feed = cv2.VideoCapture("rtsp://rinao:unicorn@192.168.1.5:8554/streaming/live/1")
cam_feed.set(cv2.CAP_PROP_FRAME_WIDTH, 650)
cam_feed.set(cv2.CAP_PROP_FRAME_HEIGHT, 750)

ws_client = None  # Global variable to hold WebSocket client

async def send_json(ws_client, data):
    json_data = json.dumps(data)
    await ws_client.send(json_data)
    print(f"Sent: {json_data}")

def for_frame(frame_number, output_array, output_count, returned_frame):
    cv2.imshow("Object Detection", returned_frame)

    if output_array:
        for obj in output_array:
            cs = obj['percentage_probability']
            target = obj['name']
            print(f"Confidence score: {cs}")
            print(f"Detected target: {target}")
            action, belief = jl.eval(f"generate_action({cs})")

            if action == 1:
                print('ALERT OPERATOR!')
                cv2.imshow("Annotated Frame", returned_frame)
            elif action == 2:
                print('GATHER INFORMATION!')
                asyncio.run(send_json(ws_client, {"action": "FlightStatus", "args": {"event": "gather-info"}}))
            elif action == 3:
                print('CONTINUE MISSION!')
            print("----------------------------------------------------")
    else:
        print("No objects detected in the current frame.")
        print("----------------------------------------------------")

    # Break the loop and close the window when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return False  # Stop processing frames

async def main():
    global ws_client
    uri = "ws://localhost:8085"
    async with websockets.connect(uri) as ws:
        ws_client = ws
        print("FALCO Awaiting data...")

        detector.detectObjectsFromVideo(
            camera_input=cam_feed,
            frames_per_second=20,
            log_progress=True,
            minimum_percentage_probability=40,
            return_detected_frame=True,
            per_frame_function=for_frame,
            save_detected_video=False
        )

        # Release the camera and close all OpenCV windows
        cam_feed.release()
        cv2.destroyAllWindows()

asyncio.run(main())