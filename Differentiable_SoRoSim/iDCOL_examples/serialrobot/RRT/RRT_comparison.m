%% RRT comparsion
clear
load("Linkages\Experimental_linkage.mat")
% Linkage = full_robot;
baxter_tree = importrobot("urdf\new_baxter.urdf");
% Linkage.CVRods{11}(2).Phi_odr = [1;1;1;1;1;1]*0;
% Linkage.CVRods{9}(2).Phi_odr = [1;1;1;1;1;1]*0;
% Linkage.CVRods{9}(2).UpdateAll;
% Linkage.CVRods{11}(2).UpdateAll;
% Linkage = Linkage.Update;

left_hand = 'left_hand';
right_hand = 'right_hand';
% ik = inverseKinematics('RigidBodyTree',baxter_tree);
joint_left = dictionary( 'left_e0',  -0.034547, ...
                        'left_e1', 2.0042, ...
                        'left_s0', -0.58664, ...
                        'left_s1', -1.137,...
                        'left_w0', 0.21857,...
                        'left_w1', -0.87636, ...
                        'left_w2', -0.12647, ...
                        'right_e0',  0.50658, ...
                        'right_e1', 1.984, ...
                        'right_s0', 0.22123, ...
                        'right_s1', -1.0741, ...
                        'right_w0', -0.14492, ...
                        'right_w1', -0.83544, ...
                        'right_w2', -0.13709);

config_Sol = dict2config1(joint_left,baxter_tree);

initial_config = config_Sol;

[q_left, q_right] = config2vector(initial_config);
% action = [[-0.7; -0.7;0.2;0;0;0.3];[0.7; -0.5;0.2;0.3;0.6;0];  [0.8; -0.4;0.2;0.3;-0.6;0]]
action =[q_left+[-0*pi/180;0;0;0;0;0;0*pi/180];q_right+[0*pi/180;0;0;0;0;0;-0*pi/180]];

%% initial_State and final state
load("RRT_initial_seed.mat")
g_init = Linkage.FwdKinematics(qu_uq_lfull(1,:),8);
g_init_left = g_init(5:8,:);
g_init_right = Linkage.FwdKinematics(qu_uq_lfull(1,:),19);
g_init_right = g_init_right(5:8,:);
g_init = [g_init_left; g_init_right];
q_a_init = [qu_uq_lfull(1,1:7)'; qu_uq_lfull(1,Linkage.ndof-6:Linkage.ndof)'];
initial_guess_start = [qu_uq_lfull(1,8:Linkage.ndof-7)';qu_uq_lfull(1,Linkage.ndof+1:Linkage.ndof+Linkage.nact)'; qu_uq_lfull(1, Linkage.ndof+Linkage.nact+1:end-2)'];


g_fin_left = Linkage.FwdKinematics(qu_uq_lfull(10,:),8);
g_fin_left = g_fin_left(5:8,:);
g_fin_right = Linkage.FwdKinematics(qu_uq_lfull(10,:),19);
g_fin_right = g_fin_right(5:8,:);
g_fin = [g_fin_left; g_fin_right];
q_a_fin = [qu_uq_lfull(10,1:7)'; qu_uq_lfull(10,Linkage.ndof-6:Linkage.ndof)'];
initial_guess_end = [qu_uq_lfull(10,8:Linkage.ndof-7)';qu_uq_lfull(10,Linkage.ndof+1:Linkage.ndof+Linkage.nact)'; qu_uq_lfull(10, Linkage.ndof+Linkage.nact+1:end-2)'];
%% check if the goal state and the end state are feasible
%% Check inverse kinematics code
q_a = Inverse_kinematics(Linkage,g_init, q_a_init)
q = [q_a(1:7); zeros(Linkage.ndof-14,1); q_a(8:14)];
plotq_urdf(Linkage,q, 0.05, baxter_tree, 1, [8,19]);
g_left =  Linkage.FwdKinematics(q,8);
g_left = g_left(5:8,:);
g_right = Linkage.FwdKinematics(q,19);
g_right = g_right(5:8,:);

drawFrame(g_left)
hold on
drawFrame(g_right)

%% 
RRT_params.MaxIter = 1000;
RRT_params.StepSize= 0.1;
RRT_params.GoalThreshold = 1e-1;
RRT_params.GoalSampleRate = 0.1;
RRT_params.EdgeCheckSteps = 10;

env.Radius= 0.6;
env.hole1 = [0.8496 0.1489 0.0766];
env.hole2 = [0.8676 -0.1506 0.0790];
env.pos_bounds = [0.3 0.9; -0.5 0.7; -0.3 0.5];
env.eul_bounds = [-pi/2 pi/2; -pi/2 pi/2; -pi/2 pi/2];
%%
plotq_urdf(Linkage, qu_uq_lfull(10,1:Linkage.ndof), 0.05, baxter_tree, 1, [8,19])
plot_constraint([env.hole1;env.hole2], env.Radius, [8,19]);
init_feasible = isFeasible(Linkage, qu_uq_lfull(10, 1:Linkage.ndof)',env)
%%
RRT_plan = RRT_planner(Linkage, g_init,g_fin,q_a_init, q_a_fin,initial_guess_start, initial_guess_end,RRT_params, env);