i_sact = cell(N, 1); %indexes of soft actuators for every link and division syntax i_sact{i}{j}
for i = 1:N
    if Linkage.VLinks(Linkage.LinkIndex(i)).linktype=='s'

        ndiv = Linkage.VLinks(Linkage.LinkIndex(i)).npie-1; 
        i_sact{i} = cell(1, ndiv);
        
        for j = 1:ndiv
            actuator_indices = zeros(1,n_sact); % if present the index otherwise zero
            for ia = 1:n_sact
                if ~isempty(dc{ia, i}) && ~isempty(dc{ia, i}{j})
                    actuator_indices(ia) = ia;
                end
            end
            
            i_sact{i}{j} = nonzeros(actuator_indices)'; % Store the non-empty ia indices for the current i, j
        end
    end
end

Linkage.i_sact = i_sact;