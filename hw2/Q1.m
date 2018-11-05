% 
% This function takes two joint configurations and the parameters of the
% obstacle as input and calculates whether a collision free path exists
% between them.
% 
% input: q1, q2 -> start and end configuration, respectively. Both are 1x4
%                  vectors.
%        sphereCenter -> 3x1 position of center of sphere
%        r -> radius of sphere
%        rob -> SerialLink class that implements the robot
% output: collision -> binary number that denotes whether this
%                      configuration is in collision or not.
function collision = Q1(rob,q1,q2,sphereCenter,r)
collision = false;
max = 25;
[~,a]=size(q1);
val = zeros(a,max);
for x=1:a
   val(x,:)=linspace(q1(x),q2(x),max); 
end
val = val';
[v,~]=size(val);
for i=1:v
collision = robotCollision(rob, val(i,:),sphereCenter,r);
if(collision) 
    break; 
end
end
end

