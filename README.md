# MPC_for_Magnetic_Levitation_System
Model Predictive Control (MPC)  for Magnetic Levitation Systems

# Description of Magnetic Levitation System

An MLS is illustrated in Fig. 1, consisting of four main components: a metal sphere (with mass $m$), an infrared sensor (transmitter and detector), a controller, and an actuator (comprising an electromagnet and power amplifier). The metal sphere is primarily controlled via the current in the electromagnetic coil $I$, which generates the electromagnetic force $F(y, I)$. This force depends on the coil current $I$ and the distance $y$, enabling the control of the metal sphere's vertical motion under the influence of both electromagnetic force and gravity.

<img src="Figures/MLS.PNG" alt="Description" width="800">

The nonlinear model of an MLS is expressed as follows:

$$
\ddot{y} = g - \frac{\hat{\psi }}{{y^2}} U^2 + \Delta(y,U,t)
$$

where $g$ is the gravitational acceleration, and $\hat{\psi }$ is the estimated value of $\psi $, where $\psi  = \frac{\chi K^2}{m}$. Here, $\chi$ is a constant depending on the electromagnet coil parameters, and $K = I/U$ is a linear relation constant between the winding current and input voltage. The term $\Delta (y, U, t) = \Delta \psi  \frac{U^2}{y^2} + D(t)$ represents the dynamic uncertainties and external disturbance, with $U$ as the control input, $\Delta \psi $ as the uncertain dynamical term, and $D(t)$ as the external disturbance.

The dynamic equation \eqref{eq3.56} can be reformulated into a second-order state-space form as follows:

$$
\left\{
\begin{array}{l}
\dot{X}_1 = X_2 \\
\dot{X}_2 = \hat{ f } + \Sigma(X,u,t) + \hat{ b }(X,t)u
\end{array}
\right.
$$

where $X_1 = y$, $X_2 = \dot{y}$, and $X = \left[ X_1 ~ X_2 \right]^T$ represents the system state vector. Additionally, $\hat{ f }(X,t) = g$, $ b (X,t) = -\frac{\hat{\psi }}{y^2}$, $\Sigma(X,u,t) = \Delta (y,U,t)$, and $u = U^2$.

Clearly, Eq. \eqref{eq3.57} represents a second-order nonlinear system.

The proposed control system can, therefore, be applied to the MLS described by Eq. \eqref{eq3.57}. With $u = U^2$, the actual control input for the MLS is given by:
\begin{equation}
U = \sqrt{u}
\end{equation}

The goal is to design an optimal control input $\tau$ that guides the actual joint position $q$ to match the target trajectory $q_d \in R^n$.

# MPC Formulation
We begin by defining the system state vector as follows:

$$
x = \begin{bmatrix} q^T \\ \dot{q}^T \end{bmatrix}^T
$$

Next, we can express the tracking error as:

$$
e = x - x_d
$$

Here, $x_d = [q^T_d \\ \dot{q}^T_d]^T$ denotes the desired trajectory, where $q_d$ and $\dot{q}_d$ represent the desired position and velocity, respectively.

MPC minimizes the following quadratic cost function over a prediction horizon \( N \):

$$
J = \sum_{k=0}^{N} \left( e ^T Q e + u_k^T R u_k \right)
$$

where:  
- $Q$ is the **state weighting matrix**.  
- $R$ is the **control weighting matrix**.  
- $N$ is the **prediction horizon**.  

The control input is optimized under the constraints:

$$
u_{\text{min}} \leq u_k \leq u_{\text{max}}
$$

MPC solves this optimization problem at every time step and applies only the first control input.

# PseudoCode

Define the number of steps N to predict ahead.

Define time step dt

1. Initial guess for control inputs u0 = zeros(n*N,1) % with n is number of joints
   
2. Define the cost function
   
  x_pred = x
  
  J = 0
  
  for k=1,...,N
  
    u_k = u(n*k-n+1:n*k)
    
    x_pred = x_pred + dt* $[\dot{q}, M^{-1}(q) \left( \tau - C(q, \dot{q}) \dot{q} - G(q) \right)]^T$
    
    e = x_pred - x_ref
    
    J = J + e' * Q * e + u_k' * R * u_k; % Compute cost
    
  end for

3.  Optimization options
4.  Solve the optimization problem
5.  Extract only the first control input
6.  Apply to robot manipulator
7.  Back to step 2

# Simulation Results and Discussion

**2-DOF Robot Manipulator**

<img src="https://github.com/user-attachments/assets/16ad4c61-c7ad-432b-a4a2-fa867f2ed659" alt="2-DOF Robot Manipulator" width="800">

**Simulation Video** 

<img src="Figures/video.gif" alt="Description" width="800">

**Simulation Results**

|_|_|
|:---:|:---:|
<img src="Figures/end-effector_tracking.png" alt="Description" width="600">| <img src="Figures/joint_tracking.png" alt="Description" width="600">|
<img src="Figures/e1.png" alt="Description" width="600">| <img src="Figures/e2.png" alt="Description" width="600">|
<img src="Figures/u1.png" alt="Description" width="600">| <img src="Figures/u2.png" alt="Description" width="600">|

**Discussion**

Examining the simulation outcomes reveals that the actual joint position trajectories closely follow the desired paths. 
Furthermore, the control signals are smooth and align with the saturation behavior of the hardware limits.

# Conclusion


# References
1. https://scholar.google.com/citations?user=Mc76hQoAAAAJ&hl=en


