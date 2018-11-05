% Calculate a path from qStart to xGoal
% input: qStart -> 1x4 joint vector describing starting configuration of
%                   arm
%        xGoal -> 3x1 position describing desired position of end q2fector
%        sphereCenter -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle
% output -> qMilestones -> 4xn vector of milestones. A straight-line interpolated
%                    path through these milestones should result in a
%                    collision-free path. You may output any number of
%                    milestones. The first milestone should be qStart. The
%                    last milestone should place the end q2fector at xGoal.
function qMilestones = Q2(rob,sphereCenter,sphereRadius,qStart,xGoal)
posInit= rob.fkine(qStart);
posInit=posInit(1:3,4); 
goal_q=inverse_kinematics(rob,qStart,xGoal);
error_goal= (xGoal-posInit);

q1(1) = Link([0 0 0 1.5963]);
q1(2) = Link([0 0 0 -1.5963]);
q1(3) = Link([0 0.471 0 -1.5963]);
r = SerialLink(q1,'name','robot');
function q = inverse_kinematics(f,qInit,xGoal)
posInit= f.fkine(qInit);
posInit=posInit(1:3,4); 
current = qInit;
error=0.001;                 
step=0.05;      
error_position= xGoal-posInit;      
while norm(error_position)>error
    current_position=f.fkine(current);     
    current_position=current_position(1:3,4);     
    jacob = jacob0(f,current);  
    jacob = jacob(1:3,1:4);
    error_position = xGoal-current_position;   
    error_q = qInit-current;       
    delta = (eye(4)-(pinv(jacob)*jacob))*error_q'+pinv(jacob)*error_position;
    current = current + (step*delta)';
end
   q=current;
end
step=0.5;
error=0.0;
run=1000;
tree_q=[qStart,0];           
error_q=inf;        
for minmaler=1:1:run
   if rand < 0.2778
        random_q=goal_q;
    else
        random_q=(2*pi*rand(1,4)-pi);
    end
    tree_current= closest_point(random_q,tree_q);
    current=tree_current(1,1:4);
    closest_q=current+(((random_q-current)/norm(random_q-current))*step);
    [collision] = collision_check(rob,r,current,closest_q,sphereCenter,sphereRadius);
    if (norm(collision.q2Pos-xGoal)<norm(error_goal))&&(collision.colsn==0)
        error_goal=collision.q2Pos-xGoal;
        parent_q=[closest_q tree_current(1,5)];
        tree_q=[tree_q;parent_q];        
        if norm(error_goal)<=error
            break;
        end
    end
end
% disp(tree_q);
qMilestones=getPath(tree_q);
disp(['qMilestones= ',num2str(size(qMilestones,1))]); 
disp(qMilestones)
end
function qMilestones=getPath(tree_q)
qMilestones=[];
child=size(tree_q,1);
while child>0    
    Q_curr=tree_q(child,1:4);
    child=tree_q(child,5);    
    qMilestones=[qMilestones;Q_curr];
end
qMilestones=flipud(qMilestones);
end
function tree_current=closest_point (random_q,tree_q)
minmal=inf;
children=size(tree_q,1);
for count=1:1:children
    dist=norm(random_q-tree_q(count,1:4));   
    if dist<minmal
        child_index=count;  
        minmal=dist;
    end
end
tree_current=[tree_q(child_index,1:4) child_index];
end
function [column]=collision_check(rob,r,current,closest_q,sphereCenter,sphereRadius)
currentq2=current;
closest_q_q2=closest_q;
closest_q_q1=closest_q(1:3);

q1_position=r.fkine(closest_q_q1);
q1_position=q1_position(1:3,4);


q2_current_position=rob.fkine(currentq2);
q2_current_position=q2_current_position(1:3,4);

q2_position=rob.fkine(closest_q_q2);
q2_position=q2_position(1:3,4);
for x1=1:1:3
    if abs(q2_position(x1))>1||abs(q1_position(x1))>1
        column.colsn=1;
        column.q2Pos=q2_current_position;
        return;
    end
end
collision=collision_points(q2_position,q1_position,q2_current_position,sphereCenter,sphereRadius);
    if collision~=1
        column.colsn=0;
    else
        column.colsn=1;
    end
column.q2Pos=q2_position;
end
function new_position=moveStep(A, B, step)
    i=(step*norm(B-A));
    new_position = A + (i*((B-A)/norm(B-A)));
end
function collision=collision_points(q2_position,q1_position,q2_current_position,sphereCenter,sphereRadius)
collision=0;
if ((norm(q2_position - sphereCenter)<sphereRadius)||(norm(q1_position - sphereCenter)<sphereRadius)||(norm((q1_position+q2_position) - sphereCenter)< sphereRadius))
    collision=1;
else
    for step=0:0.05:1
        base_q1= moveStep([0; 0; 0],q1_position,step);
        q1_q2= moveStep(q1_position,q2_position,step);
        q2_points_position= moveStep(q2_current_position,q2_position,step);
        q2_base_points_position= moveStep([0; 0; 0],(q1_position+q2_position),step);
        if ((norm(q1_q2 - sphereCenter)<sphereRadius)||(norm(base_q1 - sphereCenter)<sphereRadius)||(norm(q2_points_position - sphereCenter)<sphereRadius)||(norm(q2_base_points_position - sphereCenter)<sphereRadius))
            collision=1;
            break;
        else
            continue;
        end
    end
end
end
