# # CET-593-MMSVCC (Multimodal-Multiscale-Traffic-Control)

## About
This project involves the implementation of an MPC-based Signal-Vehicle Coupled Control (SVCC) model on a single unified 4-leg, 3-lane intersection (inspired by ***), aiming to optimize traffic flow and minimizing fuel consumption of vehicles.

## Prerequisites
Python 3.6   
GAMS 24.9  
SUMO 1.19.0  

### Installing
Pleas follow this link https://www.gams.com/latest/docs/API_PY_GETTING_STARTED.html to install the GAMS dependencies. 

## Structure
The code structure for this project can be found in /SourceCodes:  

- agent: Contains the MPC Agent Class which includes the Input, Output Interfaces, and Optimization functions.  
    - gams_models: Contains GAMS files solving the A2, A3 Optimization problems (refer to the paper [Guo and Ban (2023)](https://www.sciencedirect.com/science/article/abs/pii/S0191261523001121))  
- config: contains functions for setting up the model parameters.
- environment: Contains SUMO files of the Unified 4-leg intersection.

The whole process is illustrated in the diagram below:  

![MPC Agent Diagram](Slides/MultiScale%20Traffic%20Control%20Diagram.png)

 
Detailed documentiations can be found in /Slides/documentation.docx.  
