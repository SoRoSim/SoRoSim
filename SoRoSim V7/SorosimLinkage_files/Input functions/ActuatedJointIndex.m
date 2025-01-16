function ij_act = ActuatedJointIndex(Linkage) %compute indexes of joints that are actuated
    ij_act = zeros(Linkage.N_jact,1);
    ij = 1;
    for i=1:Linkage.N
        if ismember(i, Linkage.i_jact)
            index = find(Linkage.i_jact == i, 1);
            ij_act(index) = ij;
        end
        ij = ij+1; %rigid joint
        for j=1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1
            ij = ij+Linkage.CVRods{i}(j+1).nip-1; %soft joints
        end
    end
end