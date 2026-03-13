# 🏎️ Pure Pursuit Path Tracking Controller — 2D Simulation

![Python](https://img.shields.io/badge/Python-3.8%2B-blue?logo=python&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green)
![Status](https://img.shields.io/badge/Status-In%20Progress-yellow)

## 📖 Overview

A **from-scratch** implementation of the **Pure Pursuit** path tracking algorithm
applied to a kinematic bicycle model. This project simulates an Ackermann-steered
vehicle following a reference trajectory in 2D.

> **No frameworks. No ROS. Pure Python + Math.**

## 🚧 Work in Progress

This project is currently under development.

## 🗂️ Project Structure

    pure_pursuit_sim/
    ├── models/
    │   └── bicycle_model.py       # Kinematic bicycle model
    ├── controller/
    │   └── pure_pursuit.py        # Pure Pursuit controller
    ├── trajectory/
    │   └── path_generator.py      # Reference path generation
    ├── visualization/
    │   └── plotter.py             # Real-time plotting
    ├── tests/                     # Unit tests
    ├── docs/
    │   └── theory.md              # Mathematical derivation
    ├── main.py                    # Entry point
    ├── config.py                  # All tunable parameters
    └── requirements.txt

## 📄 License

This project is licensed under the MIT License — see [LICENSE](LICENSE) for details.