# Quadcopter-Controller
This project aim's to stabilize and control an quadcoptero and provide a study case for control and signal processing

This project will use a ARM CortexM4F (STM32F407VGT6) for implementing the main taks for a Quadcopter controller. 
The main goal of this controller is to provide enought stability and data for allowing the drone to stand still while flying
even if there ins't any user input.

Another tecnology implemented in this controller is an fusion sensor algorithm (Extendend Kalman Filter) for getting the
attitude (Quaternions or Euler Angles)  used by the controller to make the drone stable.

The controller used is an PID for each rotation axes (Roll / Pitch / Yaw) based on the ArduCoptero control algorithm, but,
this project will be used as an study case for another types of controllers (Fuzzy, SMC, ....).
