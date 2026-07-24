# Framework for the Comparison of Controller Synthesis Methods with Deadline-Miss Awareness

## Overview

This framework can be used to systematically compare controller synthesis methods for real-time control systems subject to deadline misses.
The designed controllers can be tested for varying use-cases (i.e., under different assumptions, plants, and real-time settings), and evaluated for different metrics.
The code includes three example scripts to run different case studies on both a Furuta pendulum and an electric motor.
The framework is designed to be modular and easily extendable with new controller synthesis methods, plant models and evaluation functions.

If you use this code, please cite the complementary paper:
> M. Gallant, M. Seidel, P. Pazzaglia, C. Mandrioli, C. Mark, K. Schmidt, F. Allgöwer, M. Maggio, 
> "Comparing Controller Synthesis Methods with Deadline-Miss Awareness," in IEEE Transactions on Computer-Aided Design of Integrated Circuits and Systems (TCAD), 2026 (to appear, accepted at EMSOFT 2026).

**License:** AGPL-3.0


## Requirements

The controller synthesis code requires a standard MATLAB installation.

The controllers _DR-MJLS, GraphLQR, GSS and TreeMPC_ also need the following optimisation solvers:

- [YALMIP](https://github.com/yalmip/YALMIP/releases#release-R20230622)
- [MOSEK](https://www.mosek.com/downloads/10.1.21/)

The _DMAC_ controller needs the JitterTime toolbox:

- [JitterTime](https://github.com/ControlLTH/JitterTime)

Installation procedures and setup are described on the respective webpages and need to be configured to work with MATLAB.
Make sure all required software is added to your MATLAB path by following the setup instructions.

The code was tested using MATLAB R2022 (9.13.0.2320565), YALMIP R20230622, MOSEK 10.1.21 and JitterTime 1.0.


## Setup Instructions

These instructions assume that MATLAB is already installed.

1. **Ensure that all required software/tools listed above are installed.**

Download them an place them in the preferred directory.

 * [YALMIP installation instructions](https://yalmip.github.io/tutorial/installation/)
 * [MOSEK installation instructions](https://docs.mosek.com/10.1/install/installation.html)
 * To install JitterTime you just need to clone the [repo](https://github.com/ControlLTH/JitterTime). From the command line, you can use:
   ```bash
   git clone https://github.com/ControlLTH/JitterTime.git
   ```

Then, if you haven't done it already, add the dependencies to the MATLAB path.
In the MATLAB command window, run:

   ```MATLAB
   addpath(genpath('/path/to/yalmip'));
   addpath(genpath('/path/to/mosek'));
   addpath(genpath('/path/to/jittertime'));
   savepath;
   ```

2. **Verify that YALMIP and MOSEK are recognized correctly:**

In the MATLAB command window, run:
   ```MATLAB
   yalmiptest('mosek')
   ```
   Ensure that MOSEK is listed as an available solver for SDP.

3. **Clone this framework and add it to the matlab path:**

Clone the repo with these commands from the command line:
   ```bash
   git clone https://github.com/martinamaggio/missaware-controller-synthesis
   cd missaware-controller-synthesis
   ```

Then, add it to the MATLAB path. In the MATLAB command window, run:
   ```MATLAB
   addpath(genpath('path/to/missaware-controller-synthesis'));
   savepath;
   ```

## Structure

- `controllers/`  : Controller synthesis and setup scripts for the different methods
- `data/`         : Contains the generated data
- `experimental/` : Experiment scripts for running simulation scenarios and evaluating controller performance
- `figures/`      : Contains scripts to recreate the figures of the submission based on the saved data
- `metrics/` 	  : Functions for computing various performance metrics
- `model/`        : Plant models, simulation routines, and deadline miss sequence generators


## Usage

### Reproducing the Results

In MATLAB open the folder `missaware-controller-synthesis`, then run the script:

```MATLAB
% run inside the MATLAB command window
run <name_of_script>
```

The following pre-defined scripts can be executed:
- `experimental/run_common_sequence_furuta.m` : Figures 1 - 3 of the paper are created using the data generated when running this script. This runs all controller/strategy combinations with common simulation settings for the Furuta pendulum. The run time on the tested platform is approximately 2-3 minutes.
- `experimental/run_case_study_1.m` : Figures 4 - 5 of the paper are created using the data generated when running this script. This runs the open-loop system and the selected controllers under kill/zero and skip/zero strategies with varying values of p on an electric motor. The run time on the tested platform is approximately 1.5 hours.
- `experimental/run_case_study_2.m` : Figures 6 - 7 of the paper are created using the data generated when running this script. This runs the open-loop system and the output-feedback deadline-miss-aware controllers and their nominal baseline controller with increasing upper bounds of the control task's response time on an electric motor. The run time on the tested platform is approximately 1.5-2 hours.

The expected output of the pre-defined scripts consists of the status messages in the command window at the beginning of the simulation with each listed controller and of the subsequent evaluation for each controller under all listed metrics (per function call of `do_experiment` that is called multiple times in each script).
Additionally, each call of `do_experiment` creates a MATLAB figure, plotting the simulated trajectories for each listed controller sequentially.
The two motor case study scripts also create figures showing the evaluations with respect to the metrics shown in the paper.

Running these scripts generates and saves the data (in `data/`) used for creating the figures of the accompanying paper. To recreate the figures, compile the Latex file `figures/figures.tex`.

Generating a new experiment script requires the definition of system, simulation and real-time parameters, and the lists of controllers and evaluation functions.
The function `experimental/do_experiment` runs the simulations and evaluates and saves the data.
See the pre-defined scripts for an example of how to set up an experiment script.

## Extending the Evaluation Framework

### How to Add Additional Controllers

Controller synthesis methods can be added using the setup function template `controllers/template/setup_template.m`.
The arguments of the setup function needed for the respective synthesis method are submitted in the list of controllers and as outputs the function has to return a control wrapper function that is called for each periodic control instance and the initial internal controller state.

### How to Add Additional Plants

Plant models can be added using the function templates in the folder `model/template` and `controllers/template`. The files include:
- `model/template/template_at_op.m`: Defines when the plant is close to its operating point, used in the function `metrics/metric_recovery_time.m`.
- `model/template/template_enforcedhits.m`: Defines when deadline hits are enforced, for example during the swing-up procedure under a separate nonlinear controller of the inverted pendulum.
- `model/template/template_linear.m`: Solves the linear state-space system.
- `model/template/template_matrices.m`: Defines the state-space system matrices.
- `model/template/template_nonlinear.m`: Solves the nonlinear state-space system.
- `model/template/template_plot.m`: Plots simulation results.
- `controllers/template/template_base.m`: Control base function that calls the applicable control function and can apply actuator saturation.

In order to use additional plants with pre-defined controllers, some controllers require plant-specific parameter functions to be added (`controllers/[controller]/[controller]_param_[plant].m`), where inputs and outputs depend on the respective controller.