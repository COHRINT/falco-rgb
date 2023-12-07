# RGB-FALCO

Our research focuses on designing and implementing an autonomous target detection framework that optimally alerts the operator. This alert is based on the performance history of the detector within the current mission environment and considers the limitations and capabilities of the system.

## Framework Overview

The framework comprises a RGB-detector (YOLOv3) that produce confidence scores—the scores are inputs for a Partially Observable Markov Decision Process (POMDP)-based False Alert Filter.

The False Alert Filter is responsible for commanding the operational autonomy of the robotic system during missions. Depending on the level of confidence and environmental calculus, it may instruct the robotic system to continue with the mission, gather more informative data, or alert the operator.

## Repository Structure and File Description 

Inside this repository, you will find a variety of files necessary to get our project framework up and running. Here's a quick guide to navigate them:

- **falco_function.jl**: This julia file contains the POMDP-based False Alert Filter which takes the confidence score as an input. That function is called in the Python script processing videos and detecting targets.
The `generate_action()` function is a critical method included in the "falco_function.jl" Julia file. This function uses the SARSOP policy from a previously computed policy stored in the `policy.out` file to generate an action based on the current belief state and observation, which in this case is the confidence score from the object detection. The function takes the confidence score as input and uses it along with the current belief state to decide on the optimal action: it can either alert the operator (action = 1), gather more information (action = 2), or continue the mission (action = 3). After determining the selected action, the function updates the belief state based on the given observation. The updated belief state and the chosen action are then returned by the function.

- **detection_imagePub.py**: This Python file performs object detection in a live video stream (from a webcam or an RTSP feed). It uses the Python package imageai and the Yolov3 model for object detection. Detected objects are displayed in the live video stream and corresponding information (object name, confidence score, bounding box) is stored in a dictionary.

- **policy.out**: The policy.out file is where the SARSOP-generated policy is saved. A POMDP policy defines the action the agent should perform (in this case, whether to alert the operator, gather more information, or continue the mission), depending on its belief state. The policy.out file essentially contains the decisions or actions that the system should take under different states and observations.

## How to launch RGB-FALCO

If you are about to write instructions in a README file, you might want to do it as follows:

1. **Clone the repository**
    ```bash
    git clone https://github.com/Abdoulaye27/falco.git
    ```
2. **Navigate to the cloned folder**
    ```bash
    cd /path/to/cloned_folder
    ```
    Replace '/path/to/cloned_folder' with the path where the cloned folder resides.
3. **Download RGB-YOLO weights in the cloned folder**
    ```bash
    https://drive.google.com/drive/folders/1dicOWSAtrVBVZ9JX94gTCl-clzZJ3_im?usp=sharing
    ```
    The YOLO weights are stored in the `yolo_weights.h5` file. Make sure to download in the cloned folder containing the following files: `falco_function.jl`, `detection_imagePub.py`, and `policy.out`.
4. **Dependencies**

   Run the following command to install all dependencies
   ```bash
   pip install -r requirements.txt
   ```
5. **Chose your camera feed input**
   
   If you want to use your default camera, do in `detection_imagePub.py`:
    ```bash
    cam_feed = cv2.VideoCapture(0)
    ```
    If you want to use Real Time Streaming Protocol (RTSP), do in `detection_imagePub.py`:
    ```bash
    cam_feed = cv2.VideoCapture("<rtsp_url>")
    ------------------------------------------------------------------------------------
    Example: rtsp_url = rtsp://rinao:unicorn@192.168.1.5:8554/streaming/live/1
    cam_feed = cv2.VideoCapture("rtsp://rinao:unicorn@192.168.1.5:8554/streaming/live/1")
    ```
    If you want to use Real Time Messaging Protocol (RTMP), do in `detection_imagePub.py`:
    ```bash
    cam_feed = cv2.VideoCapture("<rtmp_url>")
    ------------------------------------------------------------------------------------
    Example: rtsp_url = rtmp://myip:1935/myapp/mystream
    cam_feed = cv2.VideoCapture("rtmp://myip:1935/myapp/mystream")
    ```
6. **Run the autonomous detection framework**
   ```bash
   python detection_imagePub.py
   ```
   If you want to stop the program, click on the display screen then type "q" or "ESC".
   
## Requirements

Python: `Python 3.8.10`

You can find all the librairies in the `requirements.txt` file.

## Acknowledgements

<details><summary> <b>Expand</b> </summary>

* [https://github.com/OlafenwaMoses/ImageAI](https://github.com/OlafenwaMoses/ImageAI)

