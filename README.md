# Lateral-control-forAutonomous-Vehicles
Explicit Model Predictive Control for Lateral Control for Autonomous Vehicles

Autonomous lateral control of intelligent vehicles based on explicit model predictive control (E-MPC) is developed to implement optimal path tracking for lane changing scenario. For this purpose, a reference trajectory, which in practice is obtained by vehicle perception model and lane detection, was generated mathematically. Then, an implicit (online) model predictive control (I-MPC) with lateral dynamic vehicle model was proposed. I-MPC was first tuned until the satisfactory results are obtained. Then, the problem was solved as multi-parametric quadratic programs (mp-QP) to generate a set of explicit (offline) solutions. The polyhedral partition of the state space induced by the multi-parametric piecewise affine solution were generated to visualize the control scheme.  Last, both control schemes were compared based on computation time and memory occupied.
