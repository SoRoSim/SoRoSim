# Rotation
There exists many representations of rotations of rigid bodies. In Sorosim, we mainly adopt rotation matrices. These represent rotations with 9 parameters and 6 constraints. An element  \\(\mathbf{R} \in \mathbb{R}^{3\times 3}\\) is said to be a rotation matrix when it follows the follows the following constraints:
\\[\mathbf{R}^T\mathbf{R} = \mathbf{I}\\]
\\[\det(\mathbf{R}) = 1\\]
The set of all rotation matrices is called special orthogonal group \\(SO(3)\\).
A rotational matrix has 9 parameters and 6 constraints, the orientation can also be represented by 3 parameters called the exponential coordinates.

## Exponential coordinates of rotation
Consider a vector $p$ that is rotated about an axis \\(\hat{\omega}\\) by \\(\theta\\). This rotation is equivalent to a constant rate of 1 rad/s from \\(t = 0\\) to \\(t = \theta\\). The path traced by the vector will then be given by:
\\[\dot{p}(t) = \hat{\omega}\times p(0)\\]
If we represent the cross product in the skew-symmetric matrix form, we have:
\\[\dot{p}(t) = [\hat{\omega}]p(0)\\]
The solution to this differential equation is known to be of the form:
\\[p(\theta) = e^{[\hat{\omega}]\theta}p(0)\\]
==This will be used in the definition of the variable exp_map function==

# Rigid body motion
There is a direct analogy between the representation of rotation and the representation rigid body motion. Rigid body motion consists of both rotation as well as translation, it can be represented by a homogenous transformation matrix of the form:
\\[
	g  = \begin{bmatrix}
	\mathbf{R}_{3\times 3} & \mathbf{p}_{3\times 1}\\
	\mathbf{0}_{1\times3} &1\end{bmatrix}
\\] 
Where \\(\mathbf{R}\\) is the rotation, \\(\mathbf{p}\\) is the translation. The transformation matrix    