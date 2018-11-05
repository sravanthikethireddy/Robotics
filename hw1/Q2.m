% TODO: You write this function!
% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%                     effector position to reach <position>
%                     (orientation is to be ignored)
function q = Q2(f,qInit,posGoal)
q = qInit;
end_effector = [0; 0; 0];
x = f.fkine(q) * end_effector;
error = norm(posGoal - x);
while (error > 0.001)
    val = (posGoal - x);
    J = f.jacob0(q, 'trans');    
    Q = pinv(J) * val;
    q =q+Q';    
    x = f.fkine(q) * end_effector;
    x = x(1:3);    
    error = norm(posGoal - x);
end

end


