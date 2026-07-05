# Framework for the comparison of controller synthesis methods with deadline-miss awareness

## Overview

This framework can be used to compare controller synthesis methods for real-time control systems subject to deadline misses.
The designed controllers can be tested for varying use-cases (i.e., under different assumptions, plants, and real-time settings), and evaluated for different metrics.
The framework is designed to be modular and easily extendable with new controller synthesis methods, plant models and evaluation functions.

If you use this code, please cite the complementary paper:
> M. Gallant, M. Seidel, P. Pazzaglia, C. Mandrioli, C. Mark, K. Schmidt, F. Allgöwer, M. Maggio, 
> "Comparing Controller Synthesis Methods with Deadline-Miss Awareness," in ...

**License:** AGPL-3.0


## Requirements

The controller synthesis code requires a standard MATLAB installation and the following:

- [YALMIP](https://yalmip.github.io/)
- [MOSEK](https://www.mosek.com/)
- [JitterTime](https://github.com/ControlLTH/JitterTime)

Installation procedures and setup are described on the respective webpages and need to be configured to work with MATLAB.
Make sure all required software is added to your MATLAB path.

The code was tested using MATLAB R2022 (9.13.0.2320565), YALMIP R20230622, MOSEK 10.1.21 and JitterTime 1.0.


## Structure

- `controllers/`  : Controller synthesis and setup scripts for the different methods
- `data/`         : Contains the generated data
- `experimental/` : Experiment scripts for running simulation scenarios and evaluating controller performance
- `figures/`      : Contains scripts to recreate the figures of the submission based on the saved data
- `metrics/` 	  : Functions for computing various performance metrics
- `model/`        : Plant models, simulation routines, and deadline miss sequence generators


## Usage

### How to run experiments

The following pre-defined scripts can be executed:
- `experimental/run_common_sequence_furuta.m` : Figures 1 - 3 of the paper are created using the data generated when running this script. This runs all controller/strategy combinations with common simulation settings for the Furuta pendulum.
- `experimental/run_case_study_1.m` : Figures 4 - 5 of the paper are created using the data generated when running this script. This runs the open-loop system and the selected controllers under kill/zero and skip/zero strategies with varying values of p on an electric motor.
- `experimental/run_case_study_2.m` : Figures 6 - 7 of the paper are created using the data generated when running this script. This runs the open-loop system and the output-feedback deadline-miss-aware controllers and their nominal baseline controller with increasing upper bounds of the control task's response time on an electric motor.

Running these scripts generates and saves the data (in `data/`) used for creating the figures of the accompanying paper. To recreate the figures, compile the Latex file `figures/figures.tex`.

Generating a new experiment script requires the definition of system, simulation and real-time parameters, and the lists of controllers and evaluation functions.
The function `experimental/do_experiment` runs the simulations and evaluates and saves the data.
See the pre-defined scripts for an example of how to set up an experiment script.

### How to add additional controllers

Controller synthesis methods can be added using the setup function template `controllers/template/setup_template.m`.
The arguments of the setup function needed for the respective synthesis method are submitted in the list of controllers and as outputs the function has to return a control wrapper function that is called for each periodic control instance and the initial internal controller state.

### How to add additional plants

Plant models can be added using the function templates in the folder `model/template` and `controllers/template`. The files include:
- `model/template/template_at_op.m`: Defines when the plant is close to its operating point, used in the function `metrics/metric_recovery_time.m`.
- `model/template/template_enforcedhits.m`: Defines when deadline hits are enforced, for example during the swing-up procedure under a separate nonlinear controller of the inverted pendulum.
- `model/template/template_linear.m`: Solves the linear state-space system.
- `model/template/template_matrices.m`: Defines the state-space system matrices.
- `model/template/template_nonlinear.m`: Solves the nonlinear state-space system.
- `model/template/template_plot.m`: Plots simulation results.
- `controllers/template/template_base.m`: Control base function that calls the applicable control function and can apply actuator saturation.

In order to use additional plants with pre-defined controllers, some controllers require plant-specific parameter functions to be added (`controllers/[controller]/[controller]_param_[plant].m`), where inputs and outputs depend on the respective controller.