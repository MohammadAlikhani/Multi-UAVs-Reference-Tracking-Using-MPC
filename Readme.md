# Multi-UAVs Reference Tracking Using MPC

> Final project for the *Model Predictive Control* course at Universitat Politècnica de Catalunya (UPC).

This project implements **Linear** and **Nonlinear Model Predictive Control (LMPC & NLMPC)** strategies for tracking reference trajectories with a fleet of three quadrotor UAVs, considering collision avoidance, input/state constraints, and wind disturbances.

---

##  📚Table of Contents

- [Overview](#overview)
- [System Model](#system-model)
- [Controllers](#controllers)
- [Implementation](#implementation)
- [Results](#results)
- [Conclusion](#conclusion)

---

## 🧠 Overview

The objective is to compare LMPC and NLMPC approaches for trajectory tracking in a 3-UAV system under constraints, disturbances, and nonlinear dynamics. 

The controllers are evaluated using Mean Squared Error (MSE) and behavior under wind disturbances.

---

## 🌀 System Model

The UAV is simplified to a 2D quadrotor with 6 state variables and 2 control inputs (motor thrusts):

| **States** | Meaning |
|------------|---------|
| `x`, `y`   | position |
| `θ`        | pitch angle |
| `ẋ`, `ẏ`, `θ̇` | velocities |

| **Inputs** | Meaning |
|------------|---------|
| `u₁`, `u₂` | left / right motor thrust |

**Dynamics**

$$
\begin{aligned}
\ddot{x} &= \frac{1}{m}\bigl(u_1+u_2\bigr)\sin\theta \\[4pt]
\ddot{y} &= \frac{1}{m}\bigl(u_1+u_2\bigr)\cos\theta - g \\[4pt]
\ddot{\theta} &= \frac{L}{I}\,(u_1-u_2)
\end{aligned}
$$

---

## 🧮 Controllers

### 📏 Linear MPC

- Linearized around hover: `θ = 0`, `ẋ = ẏ = 0`
- Solved using **YALMIP + quadprog**
- Cost function:
  $J = \sum_{k=0}^{H_p} e_k^\top Q e_k \;+\; u_k^\top R u_k$.
- Constraints on thrust, position, velocity, and pitch

$$
\begin{aligned}
0 &\le u_1,\;u_2 \le m g \\[6pt]
|\dot{x}|,\;|\dot{y}| &\le 2 \ \text{m/s} \\[6pt]
|\theta| &\le 0.1 \ \text{rad}, \qquad
|\dot{\theta}| \le \tfrac{\pi}{2} \ \text{rad/s}
\end{aligned}
$$


### 🌪 Nonlinear MPC

- Adds wind disturbance as drag force: `$f_{\text{ext}} = \beta\,\mathbf{v}\,\lVert\mathbf{v}\rVert$`
- Full nonlinear model solved using **CasADi + IPOPT**
- Cost includes tracking, energy, smoothness, and pitch penalties

---

## ⚙️ Implementation

- **LMPC**: Discretized linear model, optimized using `quadprog`
- **NLMPC**: Nonlinear discrete model using Euler integration, optimized with `IPOPT`
- Simulations run in MATLAB with reference sinusoidal trajectories

---

##  📊 Results

| Controller | MSE (x) | MSE (y) |
|------------|--------:|--------:|
| **LMPC**   | 0.5255  | 0.4211  |
| **NLMPC**  | 2.5661  | 0.2624  |

- LMPC performs well without disturbances
- NLMPC compensates for wind but shows higher MSE in `x` due to nonlinear effects
- Thrust and pitch vary among drones to handle tracking and disturbance rejection

### 📷 Sample Plots

## 📈 LMPC Performance (Q = 1, R = 1, Hp = 20)
![LMPC Performance](./NMPC/img/p2.png)

## 🎛️ LMPC Controls
![LMPC Controls](./NMPC/img/l1.jpg)

## 🌪️ NLMPC Performance (w = 2.5 m/s, –45º)
![NLMPC Performance](./NMPC/img/uu.png)

## 🚁 NLMPC – Drone 1 (Thrust & Pitch)
![NLMPC Drone 1](./NMPC/img/u3.jpg)

## 🚁 NLMPC – Drone 2 (Thrust & Pitch)
![NLMPC Drone 2](./NMPC/img/u4.jpg)

## 🚁 NLMPC – Drone 3 (Thrust & Pitch)
![NLMPC Drone 3](./NMPC/img/u5.jpg)

---

## 🧾 Conclusion

- **LMPC**: Simple, fast, effective for ideal conditions, but can't handle disturbances
- **NLMPC**: More robust and practical for real systems with disturbances and nonlinearities
- **MPC** excels when:
  - Predictive control is beneficial
  - Constraints need to be explicitly managed
  - System dynamics are known and structured

---

## Author

**Mohammad Alikhani Najafabadi**  
Model Predictive Control – UPC  
mohammad.najafabadi@estudiantat.upc.edu



