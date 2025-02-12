# The SoRoSim Toolbox

SoRoSim, or Soft Robot Simulator, is a MATLAB toolbox that uses the Geometric Variable Strain (GVS) approach to provide a unified framework for the modeling, analysis, and control of soft, rigid, and hybrid robots. The toolbox can be used to analyze open-, closed- and branched structures and allows the user to model many different external loading and actuation scenarios. The soft links are modeled as a Cosserat rod, a 1D, slender rod accounting for bend, twist, stretch, and shear. MATLAB GUI assists in creating links, their assembly, the assignment of DoFs, and the definition of external and actuation forces. The new version of the toolbox, **Differential_SoRoSim**, uses analytical derivatives of the governing equations of GVS for efficient mechanical analysis. We encourage users to make use of this powerful tool.

The examples folder of the toolbox contains some saved linkages and links for which you can run simulations.

## Startup
Simply start the toolbox typing in the MATLAB terminal:
```matlab
startup
```

## Paper and How to cite
Find the overview of the toolbox, validation, and examples of problems that can be analyzed using SoRoSim in our IEEE Robotics and Automation Magazine paper, ["SoRoSim: A MATLAB Toolbox for Hybrid Rigid-Soft Robots Based on the Geometric Variable-Strain Approach"](https://doi.org/10.1109/MRA.2022.3202488).
The Theory behind the toolbox can be found in our IJRR paper ["Reduced order modeling of hybrid soft-rigid robots using global, local, and state-dependent strain parameterization"](https://journals.sagepub.com/doi/10.1177/02783649241262333)

To cite, please refer to
```tex
@article{mathew2025reduced,
  title={Reduced order modeling of hybrid soft-rigid robots using global, local, and state-dependent strain parameterization},
  author={Mathew, Anup Teejo and Feliu-Talegon, Daniel and Alkayas, Abdulaziz Y and Boyer, Frederic and Renda, Federico},
  journal={The International Journal of Robotics Research},
  volume={44},
  number={1},
  pages={129--154},
  year={2025},
  publisher={SAGE Publications Sage UK: London, England}
}
```

This work was supported in part by US Office of Naval Research Global under Grant N62909-21-1-2033, and in part by the the Khalifa University of Science and Technology under Grants CIRA-2020-074, RC1-2018-KUCARS.

## Tutorial Videos
Useful YouTube links:
- [Paper video](https://www.youtube.com/watch?v=qDYrQroxfUk&ab_channel=IEEERoboticsandAutomationSociety);
- [Tutorial at Hamlyn Symposium on Medical Robotics](https://www.youtube.com/watch?v=bkoh8Yfq_vY&t=478s&ab_channel=HamlynSymposiumonMedicalRobotics);
- [YouTube Channel](https://www.youtube.com/watch?v=vx3uYiZuuHg&t=3s&ab_channel=ANUPTEEJOMATHEW).

Check out the Examples branch for **application-level** examples! Keep an eye out for future updates.
