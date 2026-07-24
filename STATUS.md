# ARTIFACT STATUS

We are applying for the following three artifact evaluation badges:


## Available

* The repository contains the complete codebase, plant models, controller functions, evaluation metrics, and simulation configurations required to replicate all experiments.
* This package includes comprehensive documentation detailing the code structure, requirements, installation processes, and instructions on how to use it.
* All source code and experimental datasets are publicly hosted and distributed under the OSI-approved AGPL-3.0 open-source license.


## Reviewed

* The scripts provided in the `experimental/` directory are ready to run. They automate the process of setting up simulations, executing state-space solvers, computing metrics, and saving output data.
* The repository provides automated plotting utilities (`figures/figures.tex`) that directly compile raw experimental data into the identical scientific figures presented in the accepted paper.


## Reproducible

* We provide exact pipeline scripts that correspond directly to each figure shown in our paper:
  * Running `experimental/run_common_sequence_furuta.m` regenerates all data for **Figures 1-3** (Furuta pendulum).
  * Running `experimental/run_case_study_1.m` regenerates all data for **Figures 4-5** (Electric motor open-loop vs. closed-loop under varying probability parameters).
  * Running `experimental/run_case_study_2.m` regenerates all data for **Figures 6-7** (Output-feedback controllers under varying control task response times).
* We provide a LaTeX file (`figures/figures.tex`) that takes the given or freshly regenerated data from the simulations and auto-generates the figures directly and with the same formatting as in the paper.