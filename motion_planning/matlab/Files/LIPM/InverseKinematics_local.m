% =========================================================================
% BIPED KINEMATICS & GAIT ANIMATION (FULLY FIXED & ROBUST 6-DOF)
% =========================================================================
clear; clc; close all;
clear invKinBody2Foot; % Resets persistent variables in IK solver

animateOn = true; 
speedupfactor = 12; % Frame step for smooth animation


% 1. Physical Link Lengths
L0  = 0.0226; % Base to hip offset
L1  = 0.0490; % Hip yaw to roll offset
L2  = 0.0295; % Intermediate link length
L3  = 0.0430; % Thigh / Hip pitch link
L4  = 0.0880; % Shin / Knee link
L5  = 0.0990; % Knee pitch to ankle pitch distance
L6  = 0.0300; % Foot contact offset

% Real Hip Base Height above Ground (total leg reach ~0.23m)
zModel = 0.08;  

% 2. Load or Auto-Generate Trajectory Data
if ~exist('footinfos', 'var')
    if exist('defaultfootinfos.mat', 'file')
        load('defaultfootinfos.mat');
    else
        warning('defaultfootinfos.mat not found. Generating synthetic trajectory data.');
        t = linspace(0, 2, 100);
        footinfos = cell(1, 1);
        footinfos{1}.timevec = t;
        
        % Foot trajectory matrices (6 x N): row 1=X (Lateral), row 3=Y (Forward), row 5=Z (Height)
        footinfos{1}.footleft = zeros(6, length(t));
        footinfos{1}.footleft(1,:) =  -(L0 + L1);                      % X: Fixed Lateral Offset
        footinfos{1}.footleft(3,:) =  0.03 * sin(2*pi*t);              % Y: Forward Walking Motion
        footinfos{1}.footleft(5,:) =  0.02 * max(0, sin(2*pi*t));       % Z: Swing Ground Height

        footinfos{1}.footright = zeros(6, length(t));
        footinfos{1}.footright(1,:) = (L0 + L1);                      % X: Fixed Lateral Offset
        footinfos{1}.footright(3,:) = -0.03 * sin(2*pi*t);             % Y: Forward Walking Motion
        footinfos{1}.footright(5,:) =  0.02 * max(0, sin(2*pi*t + pi));% Z: Swing Ground Height
    end
end
% Initialize RigidBodyTree
robot = rigidBodyTree('DataFormat', 'row');

% =========================================================================
% 3. MDH PARAMETER TABLE (8 Rows Per Leg: 6 Revolute Joints + 2 Fixed)
% =========================================================================
dhRight = [
    L0,      0,       0,       0;       % Row 1: Joint 1 (Hip Yaw)     - Revolute
    L1,    -pi/2,    0,        pi/2;    % Row 2: Hip Roll Offset       - Fixed
    L2,      0,       0,      -pi/2;    % Row 3: Joint 2 (Hip Roll)    - Revolute
    L3,      pi/2,    0,       0;       % Row 4: Joint 3 (Hip Pitch)   - Revolute       
    L4,      0,       0,       0;       % Row 5: Joint 4 (Knee Pitch)  - Revolute
    L5,      0,       0,       0;       % Row 6: Joint 5 (Ankle Pitch) - Revolute
    0,       pi/2,    0,       0;       % Row 7: Joint 6 (Ankle Roll)  - Revolute
    L6,      pi,       0,       0        % Row 8: Foot Offset           - Fixed
];

% Symmetry Mapping for Left Leg
dhLeft = dhRight;
dhLeft(1, 1) = -dhRight(1, 1); % Mirror base-to-hip offset (-L0)
dhLeft(2, 1) = -dhRight(2, 1); % Mirror hip-yaw-to-roll offset (-L1)

jointTypes = {'revolute', 'fixed', 'revolute', 'revolute', 'revolute', 'revolute', 'revolute', 'fixed'};

% Build Right Leg
for i = 1:length(jointTypes)
    body = rigidBody(['rightleg' num2str(i)]);
    jnt = rigidBodyJoint(['rightjnt' num2str(i)], jointTypes{i});
    setFixedTransform(jnt, dhRight(i,:), 'mdh'); 
    body.Joint = jnt;
    if i == 1
        addBody(robot, body, 'base');
    else
        addBody(robot, body, ['rightleg' num2str(i-1)]);
    end
end

% Build Left Leg
for i = 1:length(jointTypes)
    body = rigidBody(['leftleg' num2str(i)]);
    jnt = rigidBodyJoint(['leftjnt' num2str(i)], jointTypes{i});
    setFixedTransform(jnt, dhLeft(i,:), 'mdh'); 
    body.Joint = jnt;
    if i == 1
        addBody(robot, body, 'base');
    else
        addBody(robot, body, ['leftleg' num2str(i-1)]);
    end
end

showdetails(robot);

% Setup Graphics
hFig = figure('Name', 'Biped Gait Simulation', 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]); 
ax1 = subplot(1, 3, 1, 'Parent', hFig); 
ax2 = subplot(1, 3, 2, 'Parent', hFig); 
ax3 = subplot(1, 3, 3, 'Parent', hFig); 

numRevolute = sum(strcmp(jointTypes, 'revolute')); % 6 active joints per leg
qright0 = zeros(1, numRevolute); 
qleft0  = zeros(1, numRevolute); 
updateJoints(robot, qright0, qleft0, ax1, ax2, ax3);

% Base orientation transform matrix
n = [0;  0; -1]; 
s = [-1; 0; 0];  
a = [0;  1; 0];  
R = [n s a];   

% =========================================================================
% 4. SIMULATION & INVERSE KINEMATICS LOOP
% =========================================================================
for sIdx = 1:length(footinfos)
    stateL = footinfos{sIdx}.footleft([1 3 5],:); 
    stateR = footinfos{sIdx}.footright([1 3 5],:); 
    
    numIdx = size(stateL, 2); 
    jointsLeft   = zeros(numRevolute, numIdx); 
    jointsRight  = zeros(numRevolute, numIdx); 
    transMatLeft   = zeros(4, 4, numIdx); 
    transMatRight  = zeros(4, 4, numIdx); 
    
    for idx = 1:numIdx
        % --- Left Leg IK ---
        pL = stateL(:, idx); 
        pL(3) = stateL(3, idx) - zModel; % Foot position relative to hip
        transmatL = [R, pL; 0 0 0 1];
        
        qLeftRaw = invKinBody2Foot(transmatL, true); 
        qLeft = processJointVector(qLeftRaw, numRevolute); 
        
        jointsLeft(:, idx) = qLeft; 
        transMatLeft(:,:, idx) = transmatL; 
        
        % --- Right Leg IK ---
        pR = stateR(:, idx);
        pR(3) = stateR(3, idx) - zModel; % Foot position relative to hip
        transmatR = [R, pR; 0 0 0 1];
        
        qRightRaw = invKinBody2Foot(transmatR, false); 
        qRight = processJointVector(qRightRaw, numRevolute); 
        
        jointsRight(:, idx) = qRight; 
        transMatRight(:,:, idx) = transmatR; 
        
        % --- Animation ---
        if animateOn && (rem(idx, speedupfactor) == 0)
            if ~isvalid(hFig)
                disp('Simulation window closed by user.');
                break;
            end
            updateJoints(robot, qRight, qLeft, ax1, ax2, ax3);
        end
    end
    
    if ~isvalid(hFig), break; end
    
    footinfos{sIdx}.jointsleft    = jointsLeft; 
    footinfos{sIdx}.jointsright   = jointsRight; 
    footinfos{sIdx}.transmatleft  = transMatLeft; 
    footinfos{sIdx}.transmatright = transMatRight; 
end

if isvalid(hFig)
    updateJoints(robot, qRight, qLeft, ax1, ax2, ax3);
end

% =========================================================================
% 5. DATA EXTRACTION & SAVING
% =========================================================================
jAngsL = [];
jAngsR = [];
for idx = 1:length(footinfos)
    if isfield(footinfos{idx}, 'jointsleft') && isfield(footinfos{idx}, 'jointsright')
        t  = footinfos{idx}.timevec(:); 
        qL = footinfos{idx}.jointsleft';  % [N x 6]
        qR = footinfos{idx}.jointsright'; % [N x 6]
        
        jAngsL = [jAngsL; [t, qL]];
        jAngsR = [jAngsR; [t, qR]];
    end
end

if ~isempty(jAngsL) && ~isempty(jAngsR)
    [~, uniqueIdxL] = unique(jAngsL(:,1));
    jAngsL = jAngsL(uniqueIdxL, :);
    [~, uniqueIdxR] = unique(jAngsR(:,1));
    jAngsR = jAngsR(uniqueIdxR, :);
    save('jointAngs_generated.mat', 'jAngsL', 'jAngsR');
    disp('Successfully extracted and saved 7-column jAngsL and jAngsR to jointAngs_generated.mat!');
end

% =========================================================================
% HELPER FUNCTIONS
% =========================================================================
function qOut = processJointVector(qIn, numRevolute)
    qIn = wrapToPi(qIn(:)); % Wrap angles to [-pi, pi]
    if length(qIn) < numRevolute
        qOut = [qIn; zeros(numRevolute - length(qIn), 1)];
    else
        qOut = qIn(1:numRevolute);
    end
end

function updateJoints(robot, anglesright, anglesleft, ax1, ax2, ax3)
    if ~ishandle(ax1) || ~isvalid(ax1) || ...
       ~ishandle(ax2) || ~isvalid(ax2) || ...
       ~ishandle(ax3) || ~isvalid(ax3)
        return; 
    end
    
    desconfig = [anglesright(:)', anglesleft(:)']; 
       
    show(robot, desconfig, 'Parent', ax1, 'PreservePlot', false);
    view(ax1, 3); 
    title(ax1, '3D View');
    grid(ax1, 'on');
    
    show(robot, desconfig, 'Parent', ax2, 'PreservePlot', false);
    view(ax2, [0, 0]); 
    title(ax2, 'Frontal View');
    grid(ax2, 'on');
    
    show(robot, desconfig, 'Parent', ax3, 'PreservePlot', false);
    view(ax3, [90, 0]); 
    title(ax3, 'Lateral View');
    grid(ax3, 'on');
    
    drawnow;
end