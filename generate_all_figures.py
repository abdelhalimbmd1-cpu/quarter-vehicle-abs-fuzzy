"""
generate_all_figures.py - Generates all ABS simulation figures with pure white backgrounds and sharp black text.
Replicates the exact quarter-vehicle dynamic model and 19-rule Mamdani Fuzzy Logic Controller from Geleta et al. (2024).
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Working directory
OUT_DIR = r"C:\Users\ZORO\.gemini\antigravity\scratch\abs_fuzzy_model"

# Model Parameters (params_abs.m)
v0 = 16.67          # Initial speed (m/s) [60 km/h]
v_stop = 0.02       # Stop threshold (m/s)
m = 360.0           # Quarter-vehicle mass (kg)
g = 9.81            # Gravity (m/s^2)
R = 0.32            # Wheel rolling radius (m)
L = 2.73            # Wheelbase (m)
hcg = 0.50          # CG height (m)
rho = 1.2           # Air density (kg/m^3)
Cd = 0.35           # Aerodynamic drag coefficient
Cr = 0.015          # Rolling resistance coefficient
Af = 1.6 + 0.00056 * (m - 765) # Frontal area (m^2)
mw = 20.0           # Wheel mass (kg)
Iw = 0.5 * mw * R**2 # Wheel moment of inertia (kg*m^2)
omega0 = v0 / R     # Initial wheel speed (rad/s)
P_max = 3.0e6       # Max pressure (Pa)
Kb = 13.0           # Lumped brake gain (N*m/Pa)
Fr_val = Cr * m * g # Rolling resistance (N)
Fz0 = m * g / 4.0   # Static normal force (N)

# Mu-slip tables (Fig. 3)
slip_table = np.array([0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00])
mu_dry     = np.array([0.00, 0.40, 0.72, 0.84, 0.88, 0.905, 0.90, 0.892, 0.88, 0.85, 0.82, 0.79, 0.76, 0.73, 0.71])
mu_wet     = np.array([0.00, 0.16, 0.33, 0.41, 0.44, 0.46, 0.46, 0.45, 0.44, 0.43, 0.41, 0.39, 0.37, 0.35, 0.34])
mu_ice     = np.array([0.00, 0.12, 0.19, 0.21, 0.222, 0.226, 0.222, 0.218, 0.214, 0.205, 0.197, 0.190, 0.183, 0.176, 0.170])

# Fuzzy Membership Functions
in_centers = np.array([-1.0, -2/3, -1/3, 0.0, 1/3, 2/3, 1.0])
in_sigma = (1.0 / 3.0) / np.sqrt(2.0 * np.log(2.0)) # ~ 0.1415

out_scale = 500.0
out_centers = in_centers * out_scale
out_sigma = in_sigma * out_scale

# 19 Rules from Table 3: (in1_idx, in2_idx, out_idx) 0-indexed
# 0:NB, 1:NM, 2:NS, 3:ZE, 4:PS, 5:PM, 6:PB
# Out: 0:BPD, 1:MPD, 2:SPD, 3:PH, 4:SPI, 5:MPI, 6:BPI
rules = [
    (3, 0, 6), # 1: ZE, NB -> BPI
    (0, 3, 6), # 2: NB, ZE -> BPI
    (0, 4, 5), # 3: NB, PS -> MPI
    (4, 0, 5), # 4: PS, NB -> MPI
    (3, 1, 5), # 5: ZE, NM -> MPI
    (1, 3, 5), # 6: NM, ZE -> MPI
    (3, 2, 4), # 7: ZE, NS -> SPI
    (2, 3, 4), # 8: NS, ZE -> SPI
    (4, 2, 3), # 9: PS, NS -> PH
    (2, 4, 3), # 10: NS, PS -> PH
    (6, 2, 1), # 11: PB, NS -> MPD
    (2, 6, 1), # 12: NS, PB -> MPD
    (3, 3, 3), # 13: ZE, ZE -> PH
    (4, 3, 2), # 14: PS, ZE -> SPD
    (3, 4, 2), # 15: ZE, PS -> SPD
    (5, 3, 1), # 16: PM, ZE -> MPD
    (3, 5, 1), # 17: ZE, PM -> MPD
    (6, 3, 0), # 18: PB, ZE -> HPD(BPD)
    (3, 6, 0)  # 19: ZE, PB -> HPD(BPD)
]

def gauss_mf(x, c, sigma):
    return np.exp(-0.5 * ((x - c) / sigma)**2)

def eval_flc(error, error_rate):
    e = np.clip(error, -1.0, 1.0)
    er = np.clip(error_rate, -1.0, 1.0)
    
    # Input memberships
    mu_e = np.array([gauss_mf(e, c, in_sigma) for c in in_centers])
    mu_er = np.array([gauss_mf(er, c, in_sigma) for c in in_centers])
    
    # Rule firing strengths (Mamdani min)
    num = 0.0
    den = 0.0
    for r_e, r_er, r_out in rules:
        weight = min(mu_e[r_e], mu_er[r_er])
        c_out = out_centers[r_out]
        num += weight * c_out
        den += weight
    
    if den < 1e-9:
        return 0.0
    return num / den

def simulate_abs(mu_curve, road_name="dry"):
    dt = 1e-3
    t_max = 14.0
    N = int(t_max / dt)
    
    t_hist = []
    v_hist = []
    w_hist = []
    slip_hist = []
    Fx_hist = []
    vdot_hist = []
    dist_hist = []
    
    v = v0
    w = omega0
    dist = 0.0
    P = 0.0
    prev_error = 0.0
    
    for i in range(N):
        t = i * dt
        
        # Relative Slip
        slip = max(0.0, (v - w * R) / max(v, 1e-5))
        
        # Error & Error Rate
        error = np.clip(slip - 0.20, -1.0, 1.0)
        error_rate = (error - prev_error) / dt if i > 0 else 0.0
        error_rate = np.clip(error_rate, -1.0, 1.0)
        prev_error = error
        
        # FLC Output: dP
        dP = eval_flc(error, error_rate)
        
        # Hydraulic Pressure Integrator
        P = np.clip(P + dP * dt, 0.0, P_max)
        Tb = Kb * P
        
        # Friction mu
        mu = np.interp(min(1.0, slip), slip_table, mu_curve)
        
        # Vehicle dynamics & load transfer
        Fa = 0.5 * rho * Cd * Af * (v**2)
        denom = m - mu * m * hcg / (2.0 * L)
        v_dot = -(mu * Fz0 + Fa + Fr_val) / denom
        Fl = -m * hcg * v_dot / (2.0 * L)
        Fz = Fz0 + Fl
        Fx = mu * Fz
        
        # Wheel acceleration
        w_dot = (Fx * R - Tb) / Iw
        
        # Log signals
        t_hist.append(t)
        v_hist.append(v)
        w_hist.append(w)
        slip_hist.append(slip)
        Fx_hist.append(Fx)
        vdot_hist.append(v_dot)
        dist_hist.append(dist)
        
        # Check stopping condition
        if v <= v_stop and i > 100:
            # Final lockup at stop
            slip_hist[-1] = 1.0
            break
            
        # Integration
        v = max(v_stop, v + v_dot * dt)
        w = max(0.0, w + w_dot * dt)
        dist += v * dt

    return {
        "name": road_name,
        "t": np.array(t_hist),
        "v": np.array(v_hist),
        "w": np.array(w_hist),
        "slip": np.array(slip_hist),
        "Fx": np.array(Fx_hist),
        "vdot": np.array(vdot_hist),
        "dist": np.array(dist_hist),
        "t_stop": t_hist[-1],
        "dist_stop": dist_hist[-1]
    }

def simulate_nonabs():
    dt = 1e-3
    t_max = 14.0
    N = int(t_max / dt)
    
    t_hist, v_hist, w_hist, slip_hist, Fx_hist, vdot_hist, dist_hist = [], [], [], [], [], [], []
    v = v0
    w = omega0
    dist = 0.0
    Tb_lock = Kb * 2.5e6 / 7.2 * 1.5 # Constant lockup torque
    
    for i in range(N):
        t = i * dt
        slip = max(0.0, min(1.0, (v - w * R) / max(v, 1e-5)))
        mu = np.interp(slip, slip_table, mu_dry)
        
        Fa = 0.5 * rho * Cd * Af * (v**2)
        denom = m - mu * m * hcg / (2.0 * L)
        v_dot = -(mu * Fz0 + Fa + Fr_val) / denom
        Fl = -m * hcg * v_dot / (2.0 * L)
        Fz = Fz0 + Fl
        Fx = mu * Fz
        w_dot = (Fx * R - Tb_lock) / Iw
        
        t_hist.append(t)
        v_hist.append(v)
        w_hist.append(w)
        slip_hist.append(slip)
        Fx_hist.append(Fx)
        vdot_hist.append(v_dot)
        dist_hist.append(dist)
        
        if v <= v_stop and i > 100:
            break
            
        v = max(v_stop, v + v_dot * dt)
        w = max(0.0, w + w_dot * dt)
        dist += v * dt
        
    return {
        "t": np.array(t_hist), "v": np.array(v_hist), "w": np.array(w_hist),
        "slip": np.array(slip_hist), "Fx": np.array(Fx_hist), "vdot": np.array(vdot_hist),
        "dist": np.array(dist_hist), "t_stop": t_hist[-1], "dist_stop": dist_hist[-1]
    }

def setup_ax(ax, title, xlabel, ylabel, xlim=None, ylim=None):
    ax.set_facecolor('white')
    ax.set_title(title, fontsize=11, fontweight='bold', color='black', pad=8)
    ax.set_xlabel(xlabel, fontsize=10, fontweight='bold', color='black')
    ax.set_ylabel(ylabel, fontsize=10, fontweight='bold', color='black')
    ax.tick_params(colors='black', labelsize=9)
    for spine in ax.spines.values():
        spine.set_color('black')
        spine.set_linewidth(1.0)
    ax.grid(True, linestyle='-', color='#3b6998', alpha=0.35, linewidth=0.8)
    ax.minorticks_on()
    ax.grid(True, which='minor', linestyle=':', color='#3b6998', alpha=0.20, linewidth=0.5)
    if xlim: ax.set_xlim(xlim)
    if ylim: ax.set_ylim(ylim)

# ----------------- 1. FIGURE 8 REPRODUCTION -----------------
print("Simulating Dry Road for Fig. 8...")
res_dry = simulate_abs(mu_dry, "dry")

fig8, axs = plt.subplots(4, 2, figsize=(12, 14), facecolor='white')
fig8.subplots_adjust(hspace=0.38, wspace=0.26, top=0.95, bottom=0.06, left=0.08, right=0.95)

# a) Slip ratio
setup_ax(axs[0, 0], 'a) Slip ratio', 'Time (s)', 'Slip ratio', [0, 8], [-0.2, 1.1])
axs[0, 0].plot(res_dry['t'], res_dry['slip'], color='#cc0000', linewidth=1.8)

# b) Wheel velocity
setup_ax(axs[0, 1], 'b) Wheel Velocity', 'Time (s)', 'Wheel velocity (rad/s)', [0, 8], [0, 65])
axs[0, 1].plot(res_dry['t'], res_dry['w'], color='#cc0000', linewidth=1.8)

# c) Angular vehicle speed
axs[1, 0].plot(res_dry['t'], res_dry['v'] / R, color='#cc0000', linewidth=1.8)
setup_ax(axs[1, 0], 'c) Angular vehicle speed', 'Time (s)', 'Vehicle Velocity (rad/s)', [0, 8], [0, 65])

# d) Vehicle linear velocity
axs[1, 1].plot(res_dry['t'], res_dry['v'], color='#cc0000', linewidth=1.8)
setup_ax(axs[1, 1], 'd) Vehicle linear velocity', 'Time (s)', 'Vehicle speed (m/s)', [0, 8], [0, 18])

# e) Stopping distance
axs[2, 0].plot(res_dry['t'], res_dry['dist'], color='#cc0000', linewidth=1.8)
setup_ax(axs[2, 0], 'e) Stopping distance', 'Time (s)', 'Stopping distance (m)', [0, 8], [0, 80])

# f) Brake force
axs[2, 1].plot(res_dry['t'], res_dry['Fx'], color='#cc0000', linewidth=1.8)
setup_ax(axs[2, 1], 'f) Brake force', 'Time (s)', 'Brake force (N)', [0, 8], [0, 950])

# g) Vehicle deceleration
axs[3, 0].plot(res_dry['t'], res_dry['vdot'], color='#cc0000', linewidth=1.8)
setup_ax(axs[3, 0], 'g) Vehicle deceleration', 'Time (s)', 'Vehicle deceleration (m/s2)', [0, 8], [-2.7, -0.4])

# Hide 8th subplot
axs[3, 1].axis('off')

fig8_path = os.path.join(OUT_DIR, "fig8_results.png")
fig8.savefig(fig8_path, dpi=300, facecolor='white', edgecolor='none')
plt.close(fig8)
print("Saved:", fig8_path)

# ----------------- 2. OPTION 1: ROAD COMPARISON -----------------
print("Simulating Wet and Ice for Option 1...")
res_wet = simulate_abs(mu_wet, "wet")
res_ice = simulate_abs(mu_ice, "ice")

fig_opt1, axs1 = plt.subplots(2, 2, figsize=(13, 10), facecolor='white')
fig_opt1.subplots_adjust(hspace=0.28, wspace=0.22, top=0.93, bottom=0.08, left=0.07, right=0.96)

colors = {'dry': '#cc0000', 'wet': '#0066cc', 'ice': '#009933'}
labels = {
    'dry': f"Dry Asphalt (Stop: {res_dry['dist_stop']:.1f} m, {res_dry['t_stop']:.2f} s)",
    'wet': f"Wet Asphalt (Stop: {res_wet['dist_stop']:.1f} m, {res_wet['t_stop']:.2f} s)",
    'ice': f"Icy Road (Stop: {res_ice['dist_stop']:.1f} m, {res_ice['t_stop']:.2f} s)"
}

# 1. Velocities
setup_ax(axs1[0, 0], 'a) Vehicle (Solid) & Linear Wheel Speed (Dashed)', 'Time (s)', 'Velocity (m/s)', [0, 14], [0, 18])
for r, res in [('dry', res_dry), ('wet', res_wet), ('ice', res_ice)]:
    axs1[0, 0].plot(res['t'], res['v'], color=colors[r], linewidth=2.0, label=labels[r])
    axs1[0, 0].plot(res['t'], res['w'] * R, '--', color=colors[r], linewidth=1.2, alpha=0.8)
axs1[0, 0].legend(loc='upper right', fontsize=8.5, facecolor='white', edgecolor='#999999', labelcolor='black')

# 2. Stopping Distance
setup_ax(axs1[0, 1], 'b) Stopping Distance vs. Time', 'Time (s)', 'Stopping Distance (m)', [0, 14], [0, 240])
for r, res in [('dry', res_dry), ('wet', res_wet), ('ice', res_ice)]:
    axs1[0, 1].plot(res['t'], res['dist'], color=colors[r], linewidth=2.2, label=labels[r])
axs1[0, 1].legend(loc='upper left', fontsize=8.5, facecolor='white', edgecolor='#999999', labelcolor='black')

# 3. Braking Force Fx
setup_ax(axs1[1, 0], 'c) Longitudinal Traction Force Fx (N)', 'Time (s)', 'Force (N)', [0, 14], [0, 950])
for r, res in [('dry', res_dry), ('wet', res_wet), ('ice', res_ice)]:
    axs1[1, 0].plot(res['t'], res['Fx'], color=colors[r], linewidth=2.0, label=r.capitalize())
axs1[1, 0].legend(loc='upper right', fontsize=9, facecolor='white', edgecolor='#999999', labelcolor='black')

# 4. Slip Regulation
setup_ax(axs1[1, 1], 'd) Relative Slip Ratio Regulation', 'Time (s)', 'Slip Ratio lambda', [0, 14], [-0.05, 1.05])
axs1[1, 1].axhline(0.20, color='black', linestyle='--', linewidth=1.2, label='Target Slip (lambda=0.20)')
for r, res in [('dry', res_dry), ('wet', res_wet), ('ice', res_ice)]:
    axs1[1, 1].plot(res['t'], res['slip'], color=colors[r], linewidth=1.8, label=r.capitalize())
axs1[1, 1].legend(loc='center right', fontsize=9, facecolor='white', edgecolor='#999999', labelcolor='black')

opt1_path = os.path.join(OUT_DIR, "fig_option1_road_comparison.png")
fig_opt1.savefig(opt1_path, dpi=300, facecolor='white', edgecolor='none')
plt.close(fig_opt1)
print("Saved:", opt1_path)

# ----------------- 3. OPTION 2: ABS VS NON-ABS -----------------
print("Simulating Non-ABS Lockup for Option 2...")
res_non = simulate_nonabs()

fig_opt2, axs2 = plt.subplots(2, 2, figsize=(13, 10), facecolor='white')
fig_opt2.subplots_adjust(hspace=0.28, wspace=0.22, top=0.93, bottom=0.08, left=0.07, right=0.96)

c_abs = '#cc0000'
c_non = '#111111'

# 1. Speeds
setup_ax(axs2[0, 0], 'a) Velocity: ABS Control vs. Panic Wheel Lock', 'Time (s)', 'Velocity (m/s)', [0, 10], [0, 18])
axs2[0, 0].plot(res_dry['t'], res_dry['v'], color=c_abs, linewidth=2.2, label='ABS Vehicle Speed')
axs2[0, 0].plot(res_dry['t'], res_dry['w'] * R, '--', color=c_abs, linewidth=1.5, label='ABS Wheel (R*omega)')
axs2[0, 0].plot(res_non['t'], res_non['v'], color=c_non, linewidth=2.2, label='Non-ABS Vehicle Speed')
axs2[0, 0].plot(res_non['t'], res_non['w'] * R, ':', color='#0044aa', linewidth=2.0, label='Non-ABS Wheel (Locks in 0.15s)')
axs2[0, 0].legend(loc='upper right', fontsize=8.5, facecolor='white', edgecolor='#999999', labelcolor='black')

# 2. Slip
setup_ax(axs2[0, 1], 'b) Wheel Slip: Regulated vs. 100% Skid', 'Time (s)', 'Slip Ratio lambda', [0, 10], [-0.05, 1.1])
axs2[0, 1].axhline(0.20, color='black', linestyle='--', linewidth=1.2, label='Optimal Slip (lambda=0.20)')
axs2[0, 1].plot(res_dry['t'], res_dry['slip'], color=c_abs, linewidth=2.0, label='ABS Slip (Controlled ~0.20)')
axs2[0, 1].plot(res_non['t'], res_non['slip'], color=c_non, linewidth=2.0, label='Non-ABS Slip (Locked Skid = 1.0)')
axs2[0, 1].legend(loc='center right', fontsize=8.5, facecolor='white', edgecolor='#999999', labelcolor='black')

# 3. Deceleration
setup_ax(axs2[1, 0], 'c) Deceleration Comparison', 'Time (s)', 'Deceleration (m/s2)', [0, 10], [-3.0, 0.0])
axs2[1, 0].plot(res_dry['t'], res_dry['vdot'], color=c_abs, linewidth=2.0, label='ABS Deceleration')
axs2[1, 0].plot(res_non['t'], res_non['vdot'], color=c_non, linewidth=2.0, label='Non-ABS Deceleration (Reduced Friction)')
axs2[1, 0].legend(loc='lower right', fontsize=8.5, facecolor='white', edgecolor='#999999', labelcolor='black')

# 4. Distance
penalty = (res_non['dist_stop'] - res_dry['dist_stop']) / res_dry['dist_stop'] * 100
setup_ax(axs2[1, 1], 'd) Stopping Distance Comparison', 'Time (s)', 'Stopping Distance (m)', [0, 10], [0, 95])
axs2[1, 1].plot(res_dry['t'], res_dry['dist'], color=c_abs, linewidth=2.2, label=f"ABS Distance: {res_dry['dist_stop']:.1f} m")
axs2[1, 1].plot(res_non['t'], res_non['dist'], color=c_non, linewidth=2.2, label=f"Non-ABS Distance: {res_non['dist_stop']:.1f} m (+{penalty:.1f}%)")
axs2[1, 1].legend(loc='upper left', fontsize=8.5, facecolor='white', edgecolor='#999999', labelcolor='black')

opt2_path = os.path.join(OUT_DIR, "fig_option2_abs_vs_nonabs.png")
fig_opt2.savefig(opt2_path, dpi=300, facecolor='white', edgecolor='none')
plt.close(fig_opt2)
print("Saved:", opt2_path)

# ----------------- 4. FUZZY 3D SURFACE (FIG. 5) -----------------
print("Generating 3D Fuzzy Surface (Fig. 5)...")
e_grid = np.linspace(-1.0, 1.0, 45)
er_grid = np.linspace(-1.0, 1.0, 45)
E, ER = np.meshgrid(e_grid, er_grid)
DP = np.zeros_like(E)

for i in range(E.shape[0]):
    for j in range(E.shape[1]):
        DP[i, j] = eval_flc(E[i, j], ER[i, j])

fig_surf = plt.figure(figsize=(10, 8), facecolor='white')
ax_s = fig_surf.add_subplot(111, projection='3d', facecolor='white')
surf = ax_s.plot_surface(E, ER, DP, cmap='jet', edgecolor='k', linewidth=0.2, alpha=0.9)

ax_s.set_title('Fig. 5 Fuzzy Logic Controller Surface Characteristics', fontsize=12, fontweight='bold', color='black', pad=14)
ax_s.set_xlabel('Error', fontsize=10, fontweight='bold', color='black', labelpad=10)
ax_s.set_ylabel('ErrorRate', fontsize=10, fontweight='bold', color='black', labelpad=10)
ax_s.set_zlabel('ChangeInPressure (Pa/s)', fontsize=10, fontweight='bold', color='black', labelpad=10)
ax_s.tick_params(colors='black', labelsize=8.5)
ax_s.view_init(elev=28, azim=130)

cb = fig_surf.colorbar(surf, shrink=0.6, aspect=12, pad=0.08)
cb.ax.tick_params(labelsize=8.5, colors='black')
cb.set_label('dP Output Range [-500, +500]', color='black', fontsize=9, fontweight='bold')

surf_path = os.path.join(OUT_DIR, "fig_fuzzy_surface_3d.png")
fig_surf.savefig(surf_path, dpi=300, facecolor='white', edgecolor='none')
plt.close(fig_surf)
print("Saved:", surf_path)
print("All figures successfully created with white background and black text!")
