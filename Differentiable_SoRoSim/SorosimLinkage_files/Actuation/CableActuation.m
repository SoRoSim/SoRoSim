%Function that allows the user to set actuation cable parameters
%Last modified: 13.01.2025, Anup Teejo Mathew
function [n_sact,dc_fn,dcp_fn,dc,dcp,Sdiv,Ediv] = CableActuation(Linkage)

prompt           = {'Number of actuators:'};
dlgtitle         = 'Actuation Parameters';
definput         = {'1'};
opts.Interpreter = 'tex';
ans_act          = inputdlg(prompt,dlgtitle,[1 50],definput,opts);
n_sact           = str2double(ans_act{1});

N         = Linkage.N;
dc_fn     = cell(n_sact,N);
dcp_fn    = cell(n_sact,N);
dc        = cell(n_sact,N);
dcp       = cell(n_sact,N);
Sdiv      = zeros(n_sact,N);
Ediv      = zeros(n_sact,N);
Linkage.n_sact = n_sact;

%Set cable parameters
for ii = 1:n_sact
    close all;

    for i=1:N
        if Linkage.VLinks(Linkage.LinkIndex(i)).linktype=='s'
            cableactuationGUI(Linkage,ii,i)
            load('cableactuation.mat','dc_fni','dcp_fni','dci','dcpi','div_starti','div_endi')
            dc_fn{ii,i}  = dc_fni;
            dcp_fn{ii,i} = dcp_fni;
            dc{ii,i}     = dci;
            dcp{ii,i}    = dcpi;
            Ediv(ii,i)   = div_endi;
            Sdiv(ii,i)   = div_starti;
        end
    end

end

end
