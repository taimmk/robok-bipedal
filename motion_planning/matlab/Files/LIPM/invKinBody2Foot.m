function qOut = invKinBody2Foot(Tdes, isLeftLeg)
% INVKINBODY2FOOT  6-DOF leg inverse kinematics for the biped model built
% in the main gait script.

    %% ---- Link lengths: MUST match the main script exactly ----
    persistent L0 L1 L2 L3 L4 L5 L6
    if isempty(L0)
        L0 = 0.0226; % Base to hip offset
        L1 = 0.0485; % Hip yaw to roll offset
        L2 = 0.0295; % Intermediate link length
        L3 = 0.0430; % Thigh / Hip pitch link
        L4 = 0.0880; % Shin / Knee link
        L5 = 0.0990; % Knee pitch to ankle pitch distance
        L6 = 0.0300; % Foot contact offset
    end

    %% ---- Warm-start state (reset by `clear invKinBody2Foot`) ----
    persistent qPrevLeft qPrevRight
    if isempty(qPrevLeft),  qPrevLeft  = zeros(6,1); end
    if isempty(qPrevRight), qPrevRight = zeros(6,1); end
    
    if isLeftLeg
        q0 = qPrevLeft;
    else
        q0 = qPrevRight;
    end

    %% ---- Solve ----
    [qSol, finalErr] = solveIKRobust(Tdes, isLeftLeg, q0, L0,L1,L2,L3,L4,L5,L6);
    
    if finalErr > 1e-2 
        warning('invKinBody2Foot:poorConvergence', ...
            'IK residual %.3g. Target pose is near or slightly outside maximum leg reach.', finalErr);
    end

    %% ---- Store warm-start state for next call ----
    if isLeftLeg
        qPrevLeft = qSol;
    else
        qPrevRight = qSol;
    end
    qOut = qSol(:); 
end

% =========================================================================
function [qBest, errBest] = solveIKRobust(Tdes, isLeft, q0, L0,L1,L2,L3,L4,L5,L6)
    % FIXED: The safe seed now uses a negative knee bend (-0.4) 
    % to match the required bending direction of your MDH table.
    safeSeed = [0; 0; 0.2; -0.4; 0.2; 0];
    seeds = {q0, safeSeed}; 
    
    qBest = q0;
    errBest = inf;
    for k = 1:numel(seeds)
        [qSol, err] = solveIK_LM(Tdes, isLeft, seeds{k}, L0,L1,L2,L3,L4,L5,L6);
        if err < errBest
            errBest = err;
            qBest = qSol;
        end
        if errBest < 1e-5 
            break; 
        end
    end
end

% =========================================================================
function [q, errNorm] = solveIK_LM(Tdes, isLeft, q0, L0,L1,L2,L3,L4,L5,L6)
% Damped least-squares (Levenberg-Marquardt) solve
    maxIter   = 100;
    tolCost   = 1e-12;   
    lambda    = 1e-3;
    maxStep   = 0.2;     
    
    % FIXED: Joint 4 (Knee) limits are flipped to allow negative bending.
    % It is now clamped between -2.50 and -0.01.
    qMin = [-0.5; -0.5; -pi/2; -2.50; -pi/2; -pi/4];
    qMax = [ 0.5;  0.5;  pi/2; -0.01;  pi/2;  pi/4];

    q = q0(:);
    q = max(min(q, qMax), qMin); % Ensure initial guess is within limits
    
    T = legFK(q, isLeft, L0,L1,L2,L3,L4,L5,L6);
    err = poseError(Tdes, T);
    cost = err.'*err;
    
    for iter = 1:maxIter
        if cost < tolCost
            break;
        end
        J = numericJacobian(q, isLeft, L0,L1,L2,L3,L4,L5,L6);
        A = J.'*J + lambda*eye(6);
        dq = A \ (J.'*err);
        
        stepNorm = norm(dq);
        if stepNorm > maxStep
            dq = dq * (maxStep/stepNorm);
        end
        
        qNew = q + dq;
        % Clamp the joints at every step to strictly enforce limits
        qNew = max(min(qNew, qMax), qMin); 
        
        TNew = legFK(qNew, isLeft, L0,L1,L2,L3,L4,L5,L6);
        errNew = poseError(Tdes, TNew);
        costNew = errNew.'*errNew;
        
        if costNew < cost
            q = qNew; err = errNew; cost = costNew;
            lambda = max(lambda*0.5, 1e-9);
        else
            lambda = min(lambda*5.0, 1e7);
        end
    end
    errNorm = sqrt(cost);
end

% =========================================================================
function T = legFK(q, isLeft, L0,L1,L2,L3,L4,L5,L6)
    if isLeft
        a0 = -L0;
        a1 = -L1;
    else
        a0 = L0;
        a1 = L1;
    end
    a     = [a0,  a1,      L2,  L3,      L4,  L5,  0,       L6];
    alpha = [0,   -pi/2,   0,   pi/2,    0,   0,   pi/2,    pi];
    d     = [0,    0,      0,   0,       0,   0,   0,       0];
    thFix = [0,    pi/2,   0,   0,       0,   0,   0,       0]; 
    isRev = [1,    0,      1,   1,       1,   1,   1,       0];
    
    T = eye(4);
    qi = 0;
    for i = 1:8
        if isRev(i)
            qi = qi + 1;
            theta = q(qi);

            %====invert any joint===
%             if qi == 5   % Inverts the Ankle Pitch
%             theta = -theta;
%             end
        else
            theta = thFix(i);
        end
        T = T * rowTransform(a(i), alpha(i), d(i), theta);
    end
end

% =========================================================================
function T = rowTransform(a, alpha, d, theta)
    ca = cos(alpha); sa = sin(alpha);
    ct = cos(theta); st = sin(theta);
    T = [ ct,      -st,       0,      a;
          st*ca,    ct*ca,   -sa,    -sa*d;
          st*sa,    ct*sa,    ca,     ca*d;
          0,        0,        0,      1  ];
end

% =========================================================================
function e = poseError(Tdes, Tcur)
    pErr = Tdes(1:3,4) - Tcur(1:3,4);
    Rerr = Tdes(1:3,1:3) * Tcur(1:3,1:3).';
    oErr = so3Log(Rerr);
    
    % Weight position slightly higher than orientation to help leg reach
    e = [2.0 * pErr; oErr]; 
end

% =========================================================================
function w = so3Log(R)
    cosAng = (trace(R) - 1) / 2;
    cosAng = min(1, max(-1, cosAng));
    ang = acos(cosAng);
    if ang < 1e-8
        w = 0.5 * [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
    elseif abs(ang - pi) < 1e-6
        B = (R + eye(3)) / 2;
        d = diag(B);
        [~, idx] = max(d);
        axisVec = B(:,idx) / sqrt(max(d(idx), 1e-12));
        w = axisVec * ang;
    else
        axisVec = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)] / (2*sin(ang));
        w = axisVec * ang;
    end
end

% =========================================================================
function J = numericJacobian(q, isLeft, L0,L1,L2,L3,L4,L5,L6)
    h = 1e-6;
    J = zeros(6,6);
    for i = 1:6
        qp = q; qp(i) = qp(i) + h;
        qm = q; qm(i) = qm(i) - h;
        Tp = legFK(qp, isLeft, L0,L1,L2,L3,L4,L5,L6);
        Tm = legFK(qm, isLeft, L0,L1,L2,L3,L4,L5,L6);
        dPos = (Tp(1:3,4) - Tm(1:3,4)) / (2*h);
        Rrel = Tp(1:3,1:3) * Tm(1:3,1:3).';
        dRot = so3Log(Rrel) / (2*h);
        J(:,i) = [2.0 * dPos; dRot]; 
    end
end