# Geometric variable strain model
The geometric variable strain(GVS) is a generalization of the Piecewise constant curvature(PCC) **cite** model for soft bodies. In GVS, the soft body can have variable strains. Consider a slender rod-like object whose center is defined by the continous curve given by:
\\[\mathbf{g}(X) = \begin{bmatrix} \mathbf{R} && \mathbf{r} \cr \mathbf{0} &&1\end{bmatrix}\\]
Where \\(X \in [0,1]\\), \\(\mathbf{R} \in SO(3)\\) and \\(r \in \mathbb{R}^3\\).  

Infinitesimal change in space and time of \\(g\\) can be studied by introducing the following differential equations:
\\[\dot{g} = g\hat{\eta}\\]
\\[g' = g\hat{\xi}\\]


