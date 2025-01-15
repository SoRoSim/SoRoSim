%Finds the number of points at which computations are performed (significant points)
%Last modified by Anup Teejo Mathew 29.11.2024
function [nsig,nj] = find_nsig_nj(S)

nsig = S.N; %all joints
nj = S.N;
for i=1:S.N
    for j=1:S.VLinks(S.LinkIndex(i)).npie-1
        nsig = nsig+S.CVRods{i}(j+1).nip;
        nj = nj+S.CVRods{i}(j+1).nip-1;
    end
    if S.VLinks(S.LinkIndex(i)).linktype=='r'
        nsig = nsig+1;
    end
end

end