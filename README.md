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

## How to launch

If you are about to write instructions in a README file, you might want to do it as follows:

1. **Clone the repository**
    ```bash
    git clone https://github.com/Abdoulaye27/falco.git
    ```
