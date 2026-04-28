import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# ── Load data ─────────────────────────────────────────────────
true_traj = np.loadtxt('../results/true_traj.csv', delimiter=',', skiprows=1)
ekf_traj  = np.loadtxt('../results/ekf_traj.csv',  delimiter=',', skiprows=1)
odom_traj = np.loadtxt('../results/odom_traj.csv', delimiter=',', skiprows=1)
gps_meas  = np.loadtxt('../results/gps_meas.csv',  delimiter=',', skiprows=1)
errors    = np.loadtxt('../results/errors.csv',     delimiter=',', skiprows=1)

# ── Plot ──────────────────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(14, 7))

# Left — trajectories
ax = axes[0]
ax.plot(true_traj[:, 0], true_traj[:, 1],
        'g-', linewidth=2, label='Ground truth')
ax.plot(odom_traj[:, 0], odom_traj[:, 1],
        color='orange', linewidth=1.5, linestyle='--', label='Odometry only (drift)')
ax.scatter(gps_meas[:, 0], gps_meas[:, 1],
           c='red', s=15, alpha=0.5, label='Noisy GPS')
ax.plot(ekf_traj[:, 0], ekf_traj[:, 1],
        'b-', linewidth=2, label='EKF estimate (C++)')
ax.set_title('EKF C++ — 2D Robot Pose Estimation', fontsize=12)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.legend()
ax.grid(True)
ax.axis('equal')

# Right — error
ax2 = axes[1]
ax2.plot(errors, 'b-', linewidth=1.5, label='EKF position error')
ax2.axhline(y=np.mean(errors), color='red', linestyle='--',
            label=f'Mean error: {np.mean(errors):.3f}m')
ax2.fill_between(range(len(errors)), errors, alpha=0.2)
ax2.set_title('EKF Position Error over Time', fontsize=12)
ax2.set_xlabel('Timestep')
ax2.set_ylabel('Position error (m)')
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.savefig('../results/ekf_cpp_result.png', dpi=150)
plt.show()

print(f"Mean error : {np.mean(errors):.3f}m")
print(f"Final error: {errors[-1]:.3f}m")
