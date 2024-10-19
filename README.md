# Kalman Filter for Extended Systems

This repository contains the implementation of a Kalman Filter applied to extended systems. The Kalman Filter is a powerful recursive algorithm used for estimating the state of dynamic systems, particularly when the system model is affected by noise. This project specifically focuses on the extended version of the filter, suitable for nonlinear system models.

## Table of Contents
- [Overview](#overview)
- [Projects](#projects)
- [Installation](#installation)
- [Usage](#usage)

## Overview

Model-based estimation techniques, such as the Kalman Filter, are critical in fields like robotics, control systems, and autonomous vehicles for providing real-time estimates of dynamic systems. In nonlinear systems, the Extended Kalman Filter (EKF) is employed by linearizing the system about the current estimate. This repository provides MATLAB-based implementations to demonstrate the functionality of the EKF in systems with nonlinear dynamics and noisy measurements.

## Projects

1. **Extended Kalman Filter for Nonlinear Systems**:
    - This project implements the EKF for a nonlinear system, demonstrating its use in dynamic state estimation. The project includes MATLAB scripts that model system dynamics, simulate noisy measurements, and apply the EKF to estimate the system’s states over time. It covers both prediction and update steps, showcasing how the filter performs with varying levels of process and measurement noise.

2. **Application to Real-World Systems**:
    - This section applies the EKF to real-world-inspired nonlinear dynamic systems, such as tracking the position and velocity of a moving object with sensor inaccuracies. The results are compared to ground truth data to illustrate the performance and accuracy of the filter in practical scenarios.

## Installation

To use the code in this repository, first clone it to your local machine:
```bash
git clone https://github.com/wasumek/Model-based-Estimation.git
```

## Usage

The MATLAB scripts implement the prediction and correction phases of the Extended Kalman Filter. Open the scripts in MATLAB and run them to simulate the nonlinear system, apply noise, and estimate the system state using the EKF. You can modify the system parameters and noise characteristics to explore how the filter adapts under different conditions.
