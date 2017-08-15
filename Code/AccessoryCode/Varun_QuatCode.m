% converts quaternion to rotation matrix
function [ output_args ] = Varun_QuatCode()

%Experiment
roll = -35.264; %x
pitch = -88.282; %y
yaw = -124.128; %z

roll = 11.116; %x
pitch = -46.824; %y
yaw = 18.046; %z

quatOr = angle2quat(roll/57.2958, pitch/57.2958, yaw/57.2958);
[a,b,c]=quat2angle(quatOr);   
[a*57.2958, b*57.2958, c*57.2958]

omega1 = 0.16059/1000;
omega2 = 10.57667/1000;
omega3 = 0.63163/1000;

omega1 = -1.33309/1000;
omega2 = 11.33412/1000;
omega3 = 0.69405/1000;

dt = 1;
N = 165;


% initOrX = 0;
% initOrY = 0;
% initOrZ = 0;
% 
% omega1 = 0/2/1000;
% omega2 = 0/2/1000;
% omega3 = 4*pi/2/1000;

omega = [omega1, omega2, omega3];

transformedAngle = zeros(N,3);

%quatOr = angle2quat(initOrX, initOrY, initOrZ);
MatrixOr = quatToMat(quatOr);

for time = 1:N
    qdot = getQdot( omega, quatOr );
    quatOr = quatOr + qdot*dt;
    quatOr = quatOr ./ sqrt(dot(quatOr,quatOr)); 
    [a,b,c]=quat2angle(quatOr);   
    transformedAngle(time,:) = [a*57.2958, b*57.2958, c*57.2958];
end
%output_args = transformedAngle;
transformedAngle(1,:)
transformedAngle(N,:)
%plot(transformedAngle(:,1))


% q = [s v] where s = scalar, v = vector
% [cos( theta/2) sin(theta/2)*u] where u = unit axis about which 
% the rotation takes place, theta = angle of rotation about axis
function [rot_m] = quatToMat( q )
s = q(1); vx = q(2); vy = q(3); vz = q(4);
rot_m = [ (1 - 2*vy*vy - 2*vz*vz), (2*vx*vy - 2*s*vz), (2*vx*vz + 2*s*vy);
          2*vx*vy + 2*s*vz, 1 - 2*vx*vx - 2*vz*vz, 2*vy*vz - 2*s*vx;
2*vx*vz - 2*s*vy, 2*vy*vz + 2*s*vx, 1 - 2*vx*vx - 2*vy*vy];

% converts angular velocity + quaternion to derivative of quaternion
function [qdot] = getQdot( w, q )
qdot = .5 * quatMult( [0 w], q);

% multiply quaternions together
% qr = q1 * q2
function [qr] = quatMult(q1, q2)
s1 = q1(1); v1 = q1(2:4);
s2 = q2(1); v2 = q2(2:4);
qr = [ s1*s2 - dot(v1,v2), s1*v2 + s2*v1 + cross( v1,v2) ];

% converts angle, axis to quaternion
function [quat] = angToQuat( angle, axis )
quat(1) = cos( angle / 2);
quat(2:4) = axis * sin(angle/2);