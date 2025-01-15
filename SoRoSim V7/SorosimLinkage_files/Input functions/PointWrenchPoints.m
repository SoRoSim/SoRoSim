%Function to precompute the significant point locations of point wrenches

function Fp_sig=PointWrenchPoints(S)

Fp_sig = zeros(S.np,1);

np = S.np;

for ip=1:np

    done = false;
    Fp_loc = S.Fp_loc{ip};
    i_sig  = 1;

    for i=1:S.N

        i_sig = i_sig+1; %joint
        if S.VLinks(S.LinkIndex(i)).linktype=='r'
            if i==Fp_loc(1)
                Fp_sig(ip) = i_sig;
                break;
            end
            i_sig = i_sig+1;
        end

        for j=1:S.VLinks(S.LinkIndex(i)).npie-1
            Xs = S.CVRods{i}(j+1).Xs;
            if i==Fp_loc(1)&&j==Fp_loc(2)
                for ii=1:S.CVRods{i}(j+1).nip
                    if Xs(ii)==Fp_loc(3)
                        Fp_sig(ip) = i_sig;
                        done = true;
                        break;
                    end
                    i_sig = i_sig+1;
                end
                if done
                    break;
                end
            else
                i_sig = i_sig+S.CVRods{i}(j+1).nip;
            end
        end
        if done
            break;
        end
    end
end

end

