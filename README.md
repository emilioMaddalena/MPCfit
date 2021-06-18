# MPCfit

Lightweight MPC approximations through tailored neural networks.

## Description :books:

We propose a particular feedforward neural network architecture to approximate model predictive controllers from data. The network features a quadratic program as one of its internal layers, employed to learn the MPC dual, as well as a final projection layer to guarantee feasibility of the control actions. It is shown that, given an appropriate network size, **any linear MPC controller can be learned**.

The so-called piecewise-affine neural network (PWA-NN) was used to learn an MPC controller for a buck dc-dc converter, and then embedded on an STM32 microcontroller. The control computations were carried out periodically in **less than 30 microseconds**. This repository contains all the files related to the project.

Evolution across epochs:

![alt text](https://github.com/emilioMaddalena/MPCfit/blob/master/files/Sketch/fitting.gif)


Final results:

<img src="https://github.com/emilioMaddalena/MPCfit/blob/master/files/Sketch/comparison.png" width="700" height="595">

## References :books:

```
@article{maddalena2021embedded,
  title={Embedded PWM predictive control of DC-DC power converters via piecewise-affine neural networks},
  author={Maddalena, E. T. and Specq, M. W. F. and Wisniewski, V. L. and Jones, C. N.},
  journal={IEEE Open Journal of the Industrial Electronics Society},
  volume={2},
  pages={199--206},
  year={2021},
  publisher={IEEE}
}
```
```
@article{maddalena2020neural,
  title={A neural network architecture to learn explicit MPC controllers from data},
  author={Maddalena, E. T. and Moraes, C. G. da S. and Waltrich, G. and Jones, C. N.},
  journal={IFAC-PapersOnLine},
  volume={53},
  number={2},
  pages={11362--11367},
  year={2020},
  publisher={Elsevier}
}
```
