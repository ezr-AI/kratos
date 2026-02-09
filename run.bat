@echo off
REM Create the conda environment once:
REM conda env create -f environment.yml

call conda activate palm-viewer
python app\main.py %*
