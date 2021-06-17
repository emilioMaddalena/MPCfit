## Folders

:open_file_folder: MATLAB: Contains the dynamics of the buck converter, the exact MPC controller code (design, explicit form and sampling). Run `model.m` before running `main.m`.

:open_file_folder: data: Stores the MPC controller samples obtained with the MATLAB scripts and also the PWA-NN models obtained with the python scripts.

:open_file_folder: python: Contains the PWA-NN python code, which reads the MPC samples and trains the PWA-NN. Example of usage `run_fit n_z`, where `n_z` is the size of the network (see associated, section IV). OBS: the PWA-NN is built on top of `qpth` (https://locuslab.github.io/qpth/).

## MATLAB dependencies

YALMIP (https://yalmip.github.io/)

MPT3 (https://www.mpt3.org/)

Gurobi (https://www.gurobi.com/downloads/)

## Python dependencies

See `requirements.txt` in `./python/`
