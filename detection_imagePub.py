import asyncio
import cv2
import websockets
import json
import os
from julia.api import Julia
from imageai.Detection import ObjectDetection

jl = Julia(compiled_modules=False)
# Load your Julia functions
jl.eval('include("falco_function.jl")')

# Initialize belief using Julia's initialize_belief()
belief = jl.eval("reset_belief()")

# The script below creates an object of the object detection class.
obj_detect = ObjectDetection()

# The next step is to set the model type for object detection. Since we’ll be using the YOLO algorithm, you need to call the setModelTypeAsYOLOv3() method as shown in the script below:
obj_detect.setModelTypeAsYOLOv3()

# The next step is to load the actual Yolo model. The Yolo model the imageai library uses for object detection is available at the following Github Link: https://bit.ly/2UqlRGD
# To load the model, first you need to call the setModelPath() method from your ObjectDetection class object and pass it the path where you downloaded the yolo.h5 model. 
# Next, you need to call the loadModel() method to actually load the model. Look at the following script for reference:
execution_path = os.getcwd()
obj_detect.setModelPath(os.path.join(execution_path, "yolov3.pt"))
obj_detect.loadModel()

# The next step is to capture your webcam stream. To do so, execute the script below:
cam_feed = cv2.VideoCapture(0)
#cam_feed = cv2.VideoCapture("rtsp://rinao:unicorn@192.168.1.5:8554/streaming/live/1")

# Next, you need to define height and width for the frame that will display the detected objects from your live feed. 
# Execute the following script to do so, recognizing you can change the integer values near the end to match your desired dimensions:
cam_feed.set(cv2.CAP_PROP_FRAME_WIDTH, 650)
cam_feed.set(cv2.CAP_PROP_FRAME_HEIGHT, 750)

async def send_json(ws_client, data):
    json_data = json.dumps(data)
    await ws_client.send(json_data)
    print(f"Sent: {json_data}")

async def main():
    uri = "ws://localhost:8085"
    async with websockets.connect(uri) as ws_client:
        print("FALCO Awaiting data...")
        count_frame = 0
        while True:
            # Reads the next frame captured by the camera
            ret, img = cam_feed.read()
            count_frame += 1
            if count_frame %5==0:
              # This function outputs the annote_image and a dictionnary of the detected object containing its name, percentage probability, and bounding boxes dimensions
              annotated_image, preds = obj_detect.detectObjectsFromImage(input_image=img,
                                output_type="array",
                                display_percentage_probability=True,
                                display_object_name=True)

              if preds and preds[0]:
                dict_obj = preds[0]
                cs = dict_obj['percentage_probability']
                target = dict_obj['name']
                print('Confidence score is {}'.format(cs))
                print('Detected target is {}'.format(target))
                if cs is None:
                  cs = 0
                action, belief = jl.eval(f"generate_action({cs})")
                action = 2
                if action == 1:
                  print('ALERT OPERATOR!')
                  cv2.imshow("", annotated_image)
                if action == 2:
                  print('GATHER INFORMATION!')
                  cv2.imshow("", annotated_image)
                  await send_json(ws_client, {"action": "FlightStatus", "args": {"event": "gather-info"}})
                if action == 3:
                  print('CONTINUE MISSION!')
                print("----------------------------------------------------")
              else:
                print("No objects detected in the current frame.")
                print("----------------------------------------------------")

            # To stop the program, press "q" or "ESC"
            if (cv2.waitKey(1) & 0xFF == ord("q")) or (cv2.waitKey(1)==27):
                break

asyncio.run(main())
cam_feed.release()
cv2.destroyAllWindows()
