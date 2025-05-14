# MuJoCo-Panthera Environment Setup

This repository provides a working setup for MuJoCo-based robotic simulation integrated with Kinova arm kinematics. It includes a set of dependencies optimized for Python 3.10 and visualization/control tools.

## Environment Setup

We recommend setting up a clean conda environment.

```bash
# Step 1: Create and activate the environment
conda create --name MuJoCo-Panthera python=3.10
conda activate MuJoCo-Panthera

# Step 2: Upgrade core tools
python -m pip install --upgrade pip setuptools wheel
```

## Dependency Installation
Install the following required packages (for now):
```bash
# Install Pinocchio from conda-forge
conda install pinocchio -c conda-forge

# Install other Python packages (note: numpy is pinned to <2.0.0 due to compatibility)
pip install mujoco dearpygui scipy numpy<2.0.0 pandas matplotlib scikit-learn PyQt6 pyserial -i https://pypi.tuna.tsinghua.edu.cn/simple
```
