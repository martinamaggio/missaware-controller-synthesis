# Setup instructions

1. **Ensure that all software/tools listed in `REQUIREMENTS.md` are installed.**

2. **Clone the repository:**
   ```bash
   git clone https://github.com/martinamaggio/missaware-controller-synthesis
   cd missaware-controller-synthesis
   ```

3. **Add the framework and dependencies to the MATLAB path:**
   ```MATLAB
   % run these commands inside the MATLAB command window
   addpath(genpath('/path/to/yalmip'));
   addpath(genpath('/path/to/mosek'));
   addpath(genpath('/path/to/jittertime'));
   addpath(genpath(pwd));
   savepath;
   ```

4. **Verify that YALMIP and MOSEK are recognized correctly:**
   ```MATLAB
   % run inside the MATLAB command window
   yalmiptest('mosek')
   ```
   Ensure that MOSEK is listed as an available solver for SDP.