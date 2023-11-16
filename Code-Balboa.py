# --- Classical imports --- 
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
sns.set()

# --- import for wheeled inverted pendulum --- 
from scipy.integrate import solve_ivp
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d
from scipy import stats

# Parameters from MATLAB code
mr = 0.316                # body part mass [kg]
mw = 2 * 0.021            # wheel(*2) mass [kg]
L = 23.0 * 1e-3           # position of COM [m]
R = 40 * 1e-3              # radius of wheel [m]

I = 444.43 * 1e-6         # inertia of body part [kg*m^2]
Iw = 2 * 26.89 * 1e-6     # inertia of wheel [kg*m^2]

Br = 0.00                 # rolling damping ratio [N*m/(rad/s)]
Bm = 0.00                 # bearing damping ratio [N*m/(rad/s)]

g = 9.81                  # gravity [m/s^2]

# Weighting matrices
F = np.array([[Br + Bm, -Bm],
              [-Bm, Bm]])
H = np.array([1, 0])

# Constants for DC motor
K = 25 / 41 * 0.506
K_τ = 48.39 / 1000
Res = 4.0
K_Res = K_τ / Res

# ---- Version 1 ----
def M(z2):
    M11 = Iw + (mw + mr) * R*R
    M12 = mr * R * L * np.cos(z2)
    M21 = mr * R * L * np.cos(z2)
    M22 = I + mr * L*L
    return np.array([[M11, M12], [M21, M22]])

# Define a function to calculate f_p(z2, z4)
def f_p(z2, z4):
    return mr * L * np.sin(z2) * np.array([ R * z4**2, g])
def F_calculator(z2, z4):
    return F @ np.array([z2,z4])

# Define the system dynamics function f(x, controller, t)
def f(x, controller, t):
    z1, z2, z3, z4 = x

    v = controller(x, t)
    T = K_Res * (v + K * z3)

    tmp = np.linalg.solve(M(z2), f_p(z2, z4) - H * T - F_calculator(z2, z4))
    return np.array([z3, z4, tmp[0], tmp[1]])



# Define the simulate_balboa function
def simulate_balboa(x0, t_list, controller, method=None):
    def system_dynamics(t, x):
        return f(x, controller, t)

    if method is None:
        sol = solve_ivp(system_dynamics, tspan, x0, t_eval=t_list)
    else:
        sol = solve_ivp(system_dynamics, tspan, x0, t_eval=t_list, method=method)

    return sol

# Read data from a CSV file
FILE = "real_data/Test_Balboa_20231031_15h18.csv"
m = np.loadtxt(FILE, delimiter=',', dtype=int, skiprows=1)

timestep = 10 * 1e-3                 # [s]
n_steps = m.shape[0]
max_time = (n_steps - 1) * timestep  # [s]
start = 0

tspan = (start, max_time)

v_list = (m[:, 0] / 1000.0) * (m[:, 1] / 400.0)  # [V]

theta_list        = np.deg2rad(m[:, 4] / 1000.0)      # [rad]
phi_list          = m[:, 2] / 4.0 / 1000.0 /R         # [m]
theta_dot_list    = np.deg2rad(m[:, 5] / 1000.0)      # [rad / s]
phi_dot_list      = m[:, 3] / 4.0 / 1000.0 /R         # [m / s]


# Define a sampled controller using interpolation
idx = int(start/timestep)
t_list = np.linspace(start, max_time, n_steps-idx)
v_list = v_list[idx:]
v_interp = interp1d(t_list, v_list, kind='linear', fill_value='extrapolate')

def controller(x, t):
    return v_interp(t)

# Initial state = [ϕ, Θ, ϕ_dot, Θ_dot]

state0 = [phi_list[idx], theta_list[idx], phi_dot_list[idx], theta_dot_list[idx]]

# Simulate the Balboa system
sol = simulate_balboa(state0, t_list, controller)

# Plot the results
plt.figure(figsize=(12, 8))

plt.subplot(2, 2, 1)
plt.plot(sol.t, sol.y[0], label='$\phi$ (ODE)')
plt.plot(t_list, phi_list[idx:], label='$\phi$ (Real data)')
plt.xlabel('Time (s)')
plt.ylabel('$\phi$ (rad)')
plt.legend()

plt.subplot(2, 2, 2)
plt.plot(sol.t, sol.y[1], label='$\Theta$ (ODE)')
plt.plot(t_list, theta_list[idx:], label='$\Theta$ (Real data)')
plt.xlabel('Time (s)')
plt.ylabel('$\Theta$ (m)')
plt.legend()

plt.subplot(2, 2, 3)
plt.plot(sol.t, sol.y[2], label='$\dot{\phi}$ (ODE)')
plt.plot(t_list, phi_dot_list[idx:], label='$\dot{\phi}$ (Real data)')
plt.xlabel('Time (s)')
plt.ylabel('$\dot{\phi}$ (rad/s)')
plt.legend()

plt.subplot(2, 2, 4)
plt.plot(sol.t, sol.y[3], label='$\dot{x}$ (ODE)')
plt.plot(t_list, theta_dot_list[idx:], label='$\dot{\Theta}$ (Real data)')
plt.xlabel('Time (s)')
plt.ylabel('$\dot{\Theta}$ (m/s)')
plt.legend()

plt.tight_layout()
plt.savefig("plot/InvertedPendulum2.png")
plt.show()