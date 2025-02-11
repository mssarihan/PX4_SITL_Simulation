# PX4 Software in the Loop Project via Simulink

Using Simulink's 3rd party support package for PX4, SITL mode can be activated effortlessly.

This project is useful when trying new controllers because of its fast implementation.


## Steps To Use

1. Configure your Pixhawk to SITL mode as described in [Setup](https://www.mathworks.com/help/uav/px4/ref/simulator-plant-model-example.html)
2. Open 2 different sessions of MATLAB.
2. Load the project on both.
3. Run dynamic model on the 1st session. (Visualisation will automatically appear.)
4. Monitor and Tune the controller on te 2nd session.
5. Communication via UDP should make the system run. (See the animation)