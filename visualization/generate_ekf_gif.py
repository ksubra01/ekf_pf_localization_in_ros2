import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Ellipse

# -----------------------------
# Simulation parameters
# -----------------------------
NUM_FRAMES = 60

# Ground truth trajectory
gt_x = np.linspace(0, 10, NUM_FRAMES)
gt_y = np.sin(gt_x)

# EKF estimate (noisy but converging)
ekf_x = gt_x + np.random.randn(NUM_FRAMES) * 0.12
ekf_y = gt_y + np.random.randn(NUM_FRAMES) * 0.12

# Covariance growth / shrinkage
covariance_values = np.linspace(0.05, 0.35, NUM_FRAMES)

# -----------------------------
# Visualization
# -----------------------------
fig, ax = plt.subplots(figsize=(6, 6))

def update(frame):
    ax.clear()

    # Ground truth trajectory
    ax.plot(
        gt_x[:frame+1],
        gt_y[:frame+1],
        linestyle="--",
        label="Ground Truth Trajectory"
    )

    # Ground truth pose
    ax.scatter(
        gt_x[frame],
        gt_y[frame],
        marker="x",
        s=80,
        label="Ground Truth Pose"
    )

    # EKF trajectory
    ax.plot(
        ekf_x[:frame+1],
        ekf_y[:frame+1],
        label="EKF Estimated Trajectory"
    )

    # EKF current estimate
    ax.scatter(
        ekf_x[frame],
        ekf_y[frame],
        label="EKF Estimate"
    )

    # Covariance ellipse
    ellipse = Ellipse(
        (ekf_x[frame], ekf_y[frame]),
        width=covariance_values[frame],
        height=covariance_values[frame] * 0.6,
        fill=False,
        label="EKF Covariance"
    )
    ax.add_patch(ellipse)

    ax.set_xlim(-1, 11)
    ax.set_ylim(-3, 3)
    ax.set_xlabel("X Position [m]")
    ax.set_ylabel("Y Position [m]")
    ax.set_title("Extended Kalman Filter Localization")
    ax.legend(loc="upper right")

ani = animation.FuncAnimation(
    fig,
    update,
    frames=NUM_FRAMES,
    interval=100
)

ani.save("ekf_demo_final.gif", writer="pillow")
plt.close()

print("Saved ekf_demo_final.gif")
