function sys = get_motor_response(params)
    %% Unpack the parameters
R = params(1);
L = params(2);
Km = params(3);
Kb = params(4);
Kf = params(5);
J = params(6);
% for the controller
q1= params(7);
q2= params(8);
r= params(9);
   %% State-space model of the DC motor
h1 = tf(Km,[L R]);            % armature
h2 = tf(1,[J Kf]);            % eqn of motion
dcm = ss(h2) * [h1 , 1];      % w = h2 * (h1*Va + Td)
dcm = feedback(dcm,Kb,1,1);   % close back emf loop
%% LQR DC Motor Control Design
dc_aug = [1 ; tf(1,[1 0])] * dcm(1); % add output w/s to DC motor model
Klqr = lqry(dc_aug, [q1 0; 0 q2], r);
P = augstate(dcm);                     % inputs:Va,Td  outputs:w,x
C = Klqr * append(tf(1,[1 0]),1,1);   % compensator including 1/s
OL = P * append(C,1);                  % open loop
CL = feedback(OL,eye(3),1:3,1:3);      % close feedback loops
sys = CL(1,[1 4]);  
end