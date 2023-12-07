# RGB-FALCO

Our research focuses on designing and implementing an autonomous target detection framework that optimally alerts the operator. This alert is based on the performance history of the detector within the current mission environment and considers the limitations and capabilities of the system.

## Framework Overview

The framework comprises a RGB-detector (YOLOv3) that produce confidence scores—the scores are inputs for a Partially Observable Markov Decision Process (POMDP)-based False Alert Filter.

The False Alert Filter is responsible for commanding the operational autonomy of the robotic system during missions. Depending on the level of confidence and environmental calculus, it may instruct the robotic system to continue with the mission, gather more informative data, or alert the operator.
