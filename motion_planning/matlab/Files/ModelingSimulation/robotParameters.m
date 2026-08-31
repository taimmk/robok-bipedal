% Walking Robot Parameters
% Copyright 2017-2019 The MathWorks, Inc.

%% Actuator type
actuatorType = 1; % 1:motion controlled, 2:torque controlled, 3:motor controlled
actstiff = 1000;
actdamp = 10;

%% Contact and friction parameters
contact_stiffness = 1e6; %400/0.001;          % Approximated at weight (N) / desired displacement (m)
contact_damping = 2e5; %contact_stiffness/10; % Tuned based on contact stiffness value
mu_s = 0.9;     % Static friction coefficient: Around that of rubber-asphalt
mu_k = 0.8;     % Kinetic friction coefficient: Lower than the static coefficient
mu_vth = 0.1;   % Friction velocity threshold (m/s)

height_plane = 0.2;
plane_z = height_plane; 
plane_x = 3;
plane_y = 50;
contact_point_radius = 0.00001; %m

%% Robot mechanical Parameters (m)
density = 1000;

leg_width = 0.054;
lower_leg_length = 0.099; 
upper_leg_length = 0.088;

foot_x = 0.044;
foot_y = 0.105;
foot_z = 0.0445;

torso_width = 0.140; 
torso_length = 0.160; 
torso_height = 0.053; 

torso_offset_height = 0.01; 
torso_offset_length = 0; 

world_damping = 0;      % Translational damping for 6-DOF joint [N/m]
world_rot_damping = 0;  % Rotational damping for 6-DOF joint [N*m/(rad/s)]

%% Initial conditions
% Height of the 6-DOF joint between the ground and robot torso
init_height = lower_leg_length + upper_leg_length + torso_offset_height ...
              + foot_z/2 + plane_z/2;  
% Joint angles [hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll]
init_angs_R = zeros(6,1);
init_angs_L = zeros(6,1);
          
%% Robot joint parameters
joint_damping = 1;
motion_time_constant = 0.001;

%% Joint controller parameters
hip_servo_kp = 12500;
hip_servo_ki = 3500;
hip_servo_kd = 100;

knee_servo_kp = 10000;
knee_servo_ki = 2750;
knee_servo_kd = 75;

ankle_servo_kp = 7500;
ankle_servo_ki = 2000;
ankle_servo_kd = 50;

deriv_filter_coeff = 1000;
max_torque = 1.5;

%% Electric motor parameters
motor_voltage = 24;
motor_resistance = 8;
motor_constant = 0.16;
motor_inertia = 1e-5;
motor_damping = 1e-5;
motor_inductance = 250e-6;
gear_ratio = 100;
