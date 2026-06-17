function q_a = Inverse_kinematics(Linkage,g_des,initial_guess)

    ub(1:7) = [1.70167993878; 1.047; 3.05417993878; 2.618; 3.059; 2.094; 3.059];
    lb(1:7) = [-1.70167993878; -2.147; -3.05417993878; -0.05; -3.059; -1.57079632679; -3.059];
    ub(8:14) = [1.70167993878; 1.047; 3.05417993878; 2.618; 3.059; 2.094; 3.059];
    lb(8:14) = [-1.70167993878; -2.147; -3.05417993878; -0.05; -3.059; -1.57079632679; -3.059];
    
    options = optimoptions('fmincon','Display','iter');
    q_a = fmincon(@(x)IK_Objective_function(Linkage,x, g_des), initial_guess,[], [], [], [], lb, ub, [],options);
end