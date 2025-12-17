---
title: Comprehension Checks - Chapter 2
sidebar_position: 2
---

# Comprehension Checks - Chapter 2: Physics Simulation Concepts

## Question 1
What is the standard value of gravity used in physics simulation for Earth-based robotics applications?

- A) 8.91 m/s²
- B) 9.81 m/s²
- C) 10.0 m/s²
- D) 9.18 m/s²

**Correct Answer:** B
**Explanation:** The standard value of gravity on Earth is 9.81 m/s², acting in the negative Z direction in most simulation environments. This value is critical for realistic motion and balance in humanoid robots.

## Question 2
How does decreasing the physics time step (dt) affect simulation?

- A) Increases computational performance but decreases accuracy
- B) Decreases both computational performance and accuracy
- C) Increases both computational performance and accuracy
- D) Decreases computational performance but increases accuracy

**Correct Answer:** D
**Explanation:** Decreasing the time step (e.g., from 0.01s to 0.001s) makes the simulation more accurate by capturing faster dynamics, but it requires more computation per unit time, thus decreasing performance.

## Question 3
What does "TGS" stand for in physics solvers used in Isaac Sim?

- A) Time-based Gravity Solver
- B) Two Gauss-Seidel solver
- C) Torque Generation System
- D) Total Geometric Simulation

**Correct Answer:** B
**Explanation:** TGS stands for "Two Gauss-Seidel" solver, which is NVIDIA's choice for physics simulation in Isaac Sim. It offers better stability and accuracy compared to other solver types.

## Question 4
What is the primary purpose of increasing the number of position iterations in a physics solver?

- A) To increase the rendering quality
- B) To allow faster simulation speeds
- C) To improve the accuracy of constraint solving
- D) To reduce the memory requirements

**Correct Answer:** C
**Explanation:** Position iterations determine how many times the physics solver attempts to resolve position constraints. Higher values result in more accurate constraint satisfaction but require more computation.

## Question 5
Which of the following is NOT a key physics parameter for humanoid robot simulation?

- A) Gravity
- B) Time step (dt)
- C) Rendering resolution
- D) Solver iteration counts

**Correct Answer:** C
**Explanation:** Rendering resolution is a visual parameter, not a physics parameter. The key physics parameters are gravity, time step, and solver settings, which directly affect how objects move and interact.

## Question 6
What happens when the physics time step is too large for the dynamics being simulated?

- A) Nothing changes in the simulation
- B) Simulation becomes more accurate
- C) Simulation may become unstable with unrealistic behavior
- D) Robot controllers operate more efficiently

**Correct Answer:** C
**Explanation:** If the time step is too large relative to the fastest dynamics in the system, the simulation may become unstable, showing unrealistic oscillations, energy drift, or other numerical instabilities.

## Question 7
In the context of humanoid simulation, what do "contact properties" primarily determine?

- A) The visual appearance of objects during contact
- B) How objects respond when they come into contact
- C) The rendering speed during contact events
- D) The mass of objects during contact

**Correct Answer:** B
**Explanation:** Contact properties determine how objects respond when they come into contact, including friction coefficients, restitution (bounciness), and how contact forces are computed between surfaces.

## Question 8
What is the "sim-to-real transfer" problem in robotics?

- A) The process of moving a robot from simulation to real hardware
- B) The difference between simulated and real-world robot behavior
- C) The transfer of data from simulation to reality
- D) The synchronization between simulation and real-world time

**Correct Answer:** B
**Explanation:** The "sim-to-real transfer" problem refers to the differences between how robots behave in simulation versus how they behave in the real world, which is a significant challenge in robotics development.

## Question 9
Which parameter would you most likely increase to improve the stability of a humanoid robot's balance in simulation?

- A) Gravity value
- B) Time step size
- C) Solver position iterations
- D) Rendering frame rate

**Correct Answer:** C
**Explanation:** Increasing solver position iterations improves constraint satisfaction, which enhances the stability of balance behaviors in humanoid robots by better resolving the contact and joint constraints.

## Question 10
Why is it important to accurately model mass properties (mass, center of mass, inertia) in humanoid robots?

- A) It affects only the visual appearance
- B) It impacts the computational performance
- C) It affects the robot's dynamic behavior and stability
- D) It determines the robot's maximum speed

**Correct Answer:** C
**Explanation:** Accurate mass properties are critical for realistic dynamic behavior, including how the robot responds to forces, maintains balance, and executes movements. These properties directly affect physics simulation accuracy.