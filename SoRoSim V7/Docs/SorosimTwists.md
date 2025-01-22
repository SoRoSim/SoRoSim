# SoroSimRod
The sorosimRod class defines the degrees of freedom of a joint or a soft body. In the case of a rigid joint, it creates a base \(B\) matrix that consists of 1s and 0s that indicate which dof is allowed. A soft body has 6 modes of deformation, one torsion, two bending, one elongation and two shear. Bdof depending on the order of the polynomial used to fit the [strains](../strain parametrisation), we have two important parameters of the twists class called Bdof and Bodr that 

| Type | Name      | datatype | Description                                                                                                                                             |
| ---- | --------- | -------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
|      | Type      |          | Base type (Monomial, legendre polynomial, etc)                                                                                                          |
|      | SubClass  |          | Now only for FEM Like basis (linear,quadratic,cubic)                                                                                                    |
|      | Phi_dof   |          | (6x1) array specifying the allowable DOFs of a soft piece                                                                                               |
|      | Phi_odr   |          | (6x1) array specifying the order of allowed DOFs                                                                                                        |
|      | dof       |          | degrees of freedom of each base.                                                                                                                        |
|      |           |          |                                                                                                                                                         |
|      | Phi_h     |          | Function handle for the base.                                                                                                                           |
|      | Phi       |          | (6xdof) Base matrix calculated at lumped joints or ((6xnGauss)xdof) base matrices computed at every significant points of a soft division.              |
|      | Phi_Z1    |          | Base calculated at 4th order first Zanna point (Xs + Z1*\\(\delta Xs\\))                                                                                |
|      | Phi_Z2    |          | Base calculated at 4th order second Zanna point (Xs + Z2 * \\(\delta Xs\\))                                                                             |
|      | Phi_Z     |          | Base calculated at 2nd order Zanna point.                                                                                                               |
|      |           |          |                                                                                                                                                         |
|      | xi_starfn |          | Reference strain vector as a function of X                                                                                                              |
|      | xi_star   |          | (6x1) reference strain vector at the lumped joint of ((6xnGauss)x4) reference strain vectors computed at Gauss quadrature and Zannah collocation points |
|      |           |          |                                                                                                                                                         |
|      | Link      |          | Link associated with this twist, for soft links                                                                                                         |
|      | div       |          | Division associated with this twist                                                                                                                     |
|      | nip       |          | Number of integration points including the boundaries                                                                                                   |
|      | Xs        |          | Integration points                                                                                                                                      |
|      | Ws        |          | Weights of the integration points                                                                                                                       |
|      | Ms        |          | (6nipx6) inertia matrix of the cross-section                                                                                                            |
|      | Es        |          | (6nipx6) Stiffness matrix                                                                                                                               |
|      | Gs        |          | (6nipx6) Damping matrix                                                                                                                                 |
|      |           |          |                                                                                                                                                         |
|      | Xadd      |          |                                                                                                                                                         |
|      | CI        |          |                                                                                                                                                         |
|      | CIFn      |          |                                                                                                                                                         |

## Class methods
### T = **jointRod**(L,i)


### T = **jointRod_pre**(L,i)
why does this assign J_axis and then have if statements?

### [Ms,Es,Gs]= **MEG**(Link,j,Xs)

