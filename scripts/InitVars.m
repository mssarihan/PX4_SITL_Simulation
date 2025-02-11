%% Parameter Definitions
% Author : Mert Serhat SARIHAN
% Date   : 2025.01.12

%% Quadcopter Coefficients, x Configuration
Vehicle.Airframe.mass = 1.590;                      % Total Mass [kg]
Vehicle.Airframe.inertia = [0.0213, 0, 0; 
                            0, 0.0221, 0; 
                            0, 0, 0.028];           % Airframe Inertia [kg*m^2]
Vehicle.Airframe.armLength = 0.243;                 % Moment Arm [m]
Vehicle.Airframe.dragCoeff.trans = [5.5e-04, 0, 0; 
                                    0, 5.5e-04, 0; 
                                    0, 0, 6.3e-04]; % Translational Drag Coef [N/m/s]
Vehicle.Airframe.dragCoeff.rot = [5.5e-04, 0, 0; 
                                  0, 5.5e-04, 0; 
                                  0, 0, 6.3e-04];   % Rotational Drag Coef [N/rad/s]
Vehicle.Airframe.Cdx = 0.0;                         % Non-Dimentional Drag Coefficient
Vehicle.Airframe.Cdy = 0.0;                         % Non-Dimentional Side-Force Coefficient
Vehicle.Airframe.diameter = 0.243;                  % Airframe Diameter [m]
Vehicle.Motor.b = 2.02e-07;                         % Thrust Coef [N/rpm^2]
Vehicle.Motor.d = 4.18e-09;                         % Drag Coef [N*m/rpm^2]
Vehicle.Motor.hubInertia = 6.8e-05;                 % Motor Inertia [kg*m^2]
Vehicle.Motor.Bandwidth = 20;                       % Motor Bandwidth [rad/s]
Vehicle.Motor.maxRPM = 6250;                        % Motor Max. RPM [rpm]
Vehicle.Motor.minRPM = 0;                           % Motor Min. RPM [rpm]

%% Contact model
contact.translation.spring = 3100;
contact.translation.damper = 100;
contact.translation.friction = 0.5;
contact.translation.vd = 0.02;
contact.translation.maxFriction = 20;
contact.translation.maxNormal = 200;

contact.rotation.spring = 2;
contact.rotation.damper = 1;
contact.rotation.friction = 0.03;
control.rotation.maxMoment = 0.1;
control.rotation.friction = 0.025;
control.rotation.vd = 0.2;

%% Control Mixer
Controller.mechanicalMapper = [1, 1, 1, 1;
                               -1, 1, 1, -1;
                               1, -1, 1, -1;                                 
                               -1, -1, 1, 1];

Controller.invMechanicalMapper = inv(Controller.mechanicalMapper);

Controller.motorMapper = [Vehicle.Motor.b, ...
                          Vehicle.Motor.b, ...
                          Vehicle.Motor.b, ...
                          Vehicle.Motor.b;

                          -Vehicle.Motor.b*Vehicle.Airframe.armLength/sqrt(2), ...
                          Vehicle.Motor.b*Vehicle.Airframe.armLength/sqrt(2), ...
                          Vehicle.Motor.b*Vehicle.Airframe.armLength/sqrt(2), ...
                          -Vehicle.Motor.b*Vehicle.Airframe.armLength/sqrt(2);

                          Vehicle.Motor.b*Vehicle.Airframe.armLength/sqrt(2), ...
                          -Vehicle.Motor.b*Vehicle.Airframe.armLength/sqrt(2), ...
                          Vehicle.Motor.b*Vehicle.Airframe.armLength/sqrt(2), ...
                          -Vehicle.Motor.b*Vehicle.Airframe.armLength/sqrt(2);

                          -Vehicle.Motor.d, ...
                          -Vehicle.Motor.d, ...
                          Vehicle.Motor.d, ...
                          Vehicle.Motor.d];

Controller.invMotorMapper = inv(Controller.motorMapper);

%% Environment
Environment.g = [0 0 9.8055];
Environment.rho = 1.225;

%% Initial states
init.equilibriumZ =  Vehicle.Airframe.mass*Environment.g(3)/contact.translation.spring;
init.posNED = [0, 0, init.equilibriumZ]; % m
init.vb = [0 0 0]'; %m/s
init.euler = [0, 0, 0]'; %Roll Pitch Yaw Rads
init.angRates = [0, 0, 0]; %rad/s 

%Computed from above values
rotorPositions = zeros(3,4);
axisD = Vehicle.Airframe.armLength/sqrt(2);
rotorPositions(:,1) = [axisD, axisD, 0]';           %  3   1    ^ x
rotorPositions(:,2) = [-axisD, -axisD, 0]';         %   \ /     |
rotorPositions(:,3) = [+axisD, -axisD, 0]';         %   / \     |---> y
rotorPositions(:,4) = [-axisD, axisD, 0]';          %  2   4
rotorDir = [-1, -1, 1, 1]; %rotation direction. +ve = clockwise

%Reference location: Zurich
% This is the home position also

ref_lat = 473977420e-7;
ref_lon = 85455940e-7;
ref_height = 488;

% Sample Time
SampleTime = 1/100;


%% UAV Dynamics Data Serialization Constants

%Gain to convert m to mm
m_to_mm = 1000;

%Gain to convert uT to Gauss
uT_to_gauss = 0.01;

%Gain to convert m/s^2 to mg
ms2_to_mg = (1/9.80665)*1000;

%Gain to convert m/s to cm/s
ms_to_cms = 100;