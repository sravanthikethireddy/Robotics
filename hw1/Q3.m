% TODO: You write this function!
% input: f1 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        f2 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        qInit -> 1x11 vector denoting current joint configuration.
%                 First six joints are the arm joints. Joints 8,9 are
%                 finger joints for f1. Joints 10,11 are finger joints
%                 for f2.
%        f1Target, f2Target -> 3x1 vectors denoting the target positions
%                              each of the two fingers.
% output: q -> 1x11 vector of joint angles that cause the fingers to
%              reach the desired positions simultaneously.
%              (orientation is to be ignored)
function q = Q3(f1,f2,qInit,f1Target,f2Target)
    alpha = 0.05;
    q = qInit;
    init  = [0 0; 0 0; 0 0; 0 0; 0 0; 0 0];
    R    = [1 0 0; 0 1 0; 0 0 1];
    gp1 = [R  f1Target; 0 0 0 1];
    gp2 = [R  f2Target; 0 0 0 1];
    for i=1:100
        qf1 = [q(:, 1:7) q(:,  8: 9)];
        qf2 = [q(:, 1:7) q(:, 10:11)];
        pf1 = f1.fkine(qf1);
        pf2 = f2.fkine(qf2);
        Jf1  = f1.jacob0(qf1);
        Jf2  = f2.jacob0(qf2);        
        dXf1 = tr2delta(pf1, gp1);
        dXf2 = tr2delta(pf2, gp2); 
        dX   = [dXf1; dXf2];
        Jf1  = [Jf1(:,1:7) Jf1(:,8:9) init];
        Jf2  = [Jf2(:,1:7) init Jf2(:,8:9)];
        J    = [Jf1; Jf2];        
        p_j   = pinv(J);
        Q = alpha*p_j*dX;
        q  = q+Q';
    end

end

    
