# SorosimLink
The SorosimLink class is used to define the different links that will be used to define the robot body. The links can then be combined to form various robots. The class variables that are used to define a link is broadly divided into three types:
 - General properties
 - Geometric properties
 - Material properties
 Of these, only some of them are user defined, the others will be computed based on the user-defined input

|     Type     |   Name    |    datatype     | Description                                                                                                                                                                                               |
| :----------: | :-------: | :-------------: | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| user-defined | Jointtype |      char       | Joint type of the link: 'R' for revolute. 'P' for prismatic. 'H' for helical, ‘U’ for universal, ‘C’ for cylindrical, ‘A’ for planar, ‘S’ for spherical,<br>‘F’ for free motion, and ‘N’, for fixed joint |
| user-defined | LinkType  |      char       | Type of link: ‘s’ for soft body, ‘r’ for rigid body                                                                                                                                                       |
| user-defined |    CS     |      char       | Cross sectional shape: ‘C’ for circular and ‘R’ for rectangular, 'E' for elliptical.                                                                                                                      |
| ==computed== |   npie    |       int       | Number of pieces. For rigid links npie = 1; for soft links, npie = 1+number of divisions                                                                                               |
|              |           |                 |                                                                                                                                                                                                           |
|              |    lp     |   cell array    | length of each division of the link                                                                                                                                                                       |
|              |     L     |      float      | total length of the link                                                                                                                                                                                  |
|              |     r     | function handle | A function of x, that returns the radius of the cross section at x, \(x\in [0,1]\)                                                                                                                        |
|              |     h     | function handle | A function of x, that returns the height of the cross section at $x \in [0,1]$                                                                                                                            |
|              |     w     | function handle | A function of x, that returns the width of the of the cross section at $x\in [0,1]$                                                                                                                       |
|              |     a     | function handle | A function of x, that returns the semi-major axis of the cross section at $x\in[0,1]$                                                                                                                     |
|              |     b     | function handle | A function of x, that returns the semi-minor axis of the cross section at $x\in[0,1]$                                                                                                                     |
|              |    gi     |   \\(SE(3)\\)   | Transformation from the joint center to: (1) the center of mass for rigid link (2) center of area for soft link                                                                                           |
|              |    gf     |   \\(SE(3)\\)   | ==Transfor==                                                                                                                                                                                              |
|              |           |                 |                                                                                                                                                                                                           |
|              |     E     |      float      | Youngs modulus [Pa]                                                                                                                                                                                       |
|              |    Poi    |      float      | Poisson ratio                                                                                                                                                                                             |
|              |     G     |      float      | Shear modulus [Pa]                                                                                                                                                                                        |
|              |    Eta    |      float      | Material damping of soft link [Pa.s]                                                                                                                                                                      |
|              |    Rho    |      float      | Density of the link [kg/m^3]                                                                                                                                                                              |
|              |    Kj     |      float      | Joint stiffness matrix                                                                                                                                                                                    |
|              |    Dj     |      float      | joint damping Matrix                                                                                                                                                                                      |
|              |     M     |                 | Inertia matrix for rigid links only                                                                                                                                                                       |
|              |           |                 |                                                                                                                                                                                                           |
|              |    n_l    |                 |                                                                                                                                                                                                           |
|              |    n_r    |                 |                                                                                                                                                                                                           |
|              |   color   |                 |                                                                                                                                                                                                           |
|              |    CPF    |                 |                                                                                                                                                                                                           |
|              |  PlotFn   |                 |                                                                                                                                                                                                           |
|              |  Lscale   |                 |                                                                                                                                                                                                           |

## SorosimLink definition
### GUI based
To define a sorosim link, the GUI can be used to define some of the class variables. All the other class variables are computed. 
A link can be instantiated by:
```
L1 = SorosimLink;
```
This brings us to the GUI that prompts the user to define the user defined variables. The user defined variables are:
- Linktype
- jointtype
- CS
- Kj
- Rho
- L


## Class methods
==not required, will have something like this for sorosimlinkage==





