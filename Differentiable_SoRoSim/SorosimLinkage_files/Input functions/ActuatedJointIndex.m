function [ij_act,i_sig_act] = ActuatedJointIndex(Linkage) %compute indexes of joints that are actuated
    ij_act = zeros(Linkage.N_jact,1);
    i_sig_act = zeros(Linkage.N_jact,1);
    ij = 1;
    i_sig = 1;
    for i=1:Linkage.N
        i_sig = i_sig+1; % at X=0 to CM of rigid body or X=0 of first division of soft body
        if ismember(i, Linkage.i_jact)
            index = find(Linkage.i_jact == i, 1);
            ij_act(index) = ij;
            i_sig_act(index) = i_sig;
        end
        ij = ij+1; %rigid joint
        if Linkage.VLinks(Linkage.LinkIndex(i)).linktype=='r'
            i_sig = i_sig+1; %CM of rigid body to X=0 of next
        end
        for j=1:Linkage.VLinks(Linkage.LinkIndex(i)).npie-1
            ij = ij+Linkage.CVRods{i}(j+1).nip-1; %soft joints
            i_sig = i_sig+Linkage.CVRods{i}(j+1).nip;
        end
    end
end