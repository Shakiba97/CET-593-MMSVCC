# MMSVCC (Multimodal Multiscale Signal-Vehicle Coupled Control)

## About
This project involves the implementation of an MPC-based Signal-Vehicle Coupled Control (SVCC) model (Developed by Dr. Qiangqiang Guo (guoqq77@gmail.com)) on a single unified 4-leg, 3-lane intersection (inspired by ***), aiming to optimize traffic flow and minimizing fuel consumption of vehicles.

## Requirements
Python 3.12   
GAMS 46.5 (Download: https://www.gams.com/download/)  
SUMO 1.20.0  (Download: https://eclipse.dev/sumo/)  
traci  
sumolib  
numpy  
gamsapi  
matplotlib  

### Installation
Please follow this link https://www.gams.com/latest/docs/API_PY_GETTING_STARTED.html to install the GAMS dependencies. 

## Structure
- 'agent': Contains the MPC Agent Class which includes the Input, Output Interfaces (communication with SUMO), and Optimization functions.  
    - gams_models: Contains GAMS files solving the A2, A3 Optimization problems (refer to the paper [Guo and Ban (2023)](https://www.sciencedirect.com/science/article/abs/pii/S0191261523001121))  
- 'config': contains functions for setting up the model parameters.
- 'environment': Contains SUMO files of the Unified 4-leg intersection.

The whole process is summarized in the diagram below:  

![MPC Agent Diagram](Slides/MultiScale%20Traffic%20Control%20Diagram.png)

 
Detailed documentiations can be found in /Slides/documentation.docx.  
