function q = strain_parameters(Linkage,Node)
%Takes the node in a tree and gives just q

q = [Node.q_a(1:7); Node.x(1:Linkage.ndof-14); Node.q_a(8:14)];
end