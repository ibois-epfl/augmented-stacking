# Branch for camera sets software

Secondary branch for time-lapse code for recording augmented-stacking session:
use config.txt to set the directory as output.

## Conda installation/update
On new machines, create a conda env from the `yml` file:
```terminal
conda env create --file=environment.yml
```
To simply update the conda use:
```terminal
conda env update --name myenv --file local.yml --prune
```
