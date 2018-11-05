% Smooth path given in qMilestones
% input: qMilestones -> nx4 vector of n milestones. 
%        sphereCenter -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle
% output -> qMilestones -> 4xm vector of milestones. A straight-line interpolated
%                    path through these milestones should result in a
%                    collision-free path. You should output a number of
%                    milestones m<=n.
function qMilestonesSmoothed = Q3(rob,qMilestones,sphereCenter,sphereRadius)
L1(1) = Link([0 0 0 1.5963]);
L1(2) = Link([0 0 0 -1.5963]);
L1(3) = Link([0 0.471 0 -1.5963]);
r = SerialLink(L1,'name','robot');
qMilestonesSmoothed = path_smoothning(rob,r,qMilestones,sphereCenter,sphereRadius);
disp(['qMileStones after smoothing= ', num2str(size(qMilestonesSmoothed,1))]);
disp(qMilestonesSmoothed);
end
function new_postion=next_point(A, B, p)
    i=(p*norm(B-A));
    new_postion =(i*((B-A)/norm(B-A)))+A;
end
function qMilestonesSmoothed = path_smoothning(rob,r,qMilestones,sphereCenter,sphereRadius)
no_milestones = size(qMilestones,1);
milestone_q = qMilestones(1,:);
    for num = 1:no_milestones
        for next_num = num+1:25
        if(next_num>no_milestones)
            break;
        end
        current_q=qMilestones(num,:);
        next_q=qMilestones(next_num,:);
        collision = collision_check(rob,r,current_q,next_q,sphereCenter,sphereRadius);
        if collision~=0
            milestone_q =[milestone_q ; qMilestones(next_num-1,:)];
            num=next_num-1;
            break;
        else
            if (num+next_num)==no_milestones
                milestone_q = [milestone_q ; qMilestones(no_milestones,:)];
                num=no_milestones;
                break;
            end
        end
        end
    qMilestonesSmoothed=milestone_q;
    end
end    
function collision=collision_check(rob,r,currebt_q,qNext,sphereCenter,sphereRadius)
collision=0;
curr_q1_q=currebt_q(1:3);
current_q1_position=r.fkine(curr_q1_q);
current_q1_position=current_q1_position(1:3,4);

curr_q2_q=currebt_q;
current_q2_position=rob.fkine(curr_q2_q);
current_q2_position=current_q2_position(1:3,4);

next_q1_q=qNext(1:3);
next_q1_position=r.fkine(next_q1_q);
next_q1_position=next_q1_position(1:3,4);

next_q2_q=qNext;
next_q2_position=rob.fkine(next_q2_q);
next_q2_position=next_q2_position(1:3,4);
    for step=0:0.05:1
    q1_q3= next_point(current_q1_position,next_q1_position,step);
    q2_q3= next_point(current_q2_position,next_q2_position,step);
    base= next_point((current_q1_position+current_q2_position),(next_q1_position+next_q2_position),step);
        if ((norm(q1_q3 - sphereCenter)< sphereRadius) ||(norm(q2_q3 - sphereCenter)< sphereRadius)||(norm(base - sphereCenter) < sphereRadius)) 
            collision=1;
            break;
        else
            continue;
        end
    end    
end
