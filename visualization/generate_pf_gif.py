import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# -----------------------------
# Simulation parameters
# -----------------------------
NUM_FRAMES = 60
NUM_PARTICLES = 200

# Ground truth trajectory
gt_x = np.linspace(0, 10, NUM_FRAMES)
gt_y = np.sin(gt_x)

# Particle cloud around ground truth
particles_x = [
    gt_x[i] + np.random.randn(NUM_PARTICLES) * 0.35
    for i in range(NUM_FRAMES)
]
particles_y = [
    gt_y[i] + np.random.randn(NUM_PARTICLES) * 0.35
    for i in range(NUM_FRAMES)
]

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

    # Particle cloud
    ax.scatter(
        particles_x[frame],
        particles_y[frame],
        alpha=0.35,
        label="Particles"
    )

    ax.set_xlim(-1, 11)
    ax.set_ylim(-3, 3)
    ax.set_xlabel("X Position [m]")
    ax.set_ylabel("Y Position [m]")
    ax.set_title("Particle Filter Localization")
    ax.legend(loc="upper right")

ani = animation.FuncAnimation(
    fig,
    update,
    frames=NUM_FRAMES,
    interval=100
)

ani.save("pf_demo_final.gif", writer="pillow")
plt.close()

print("Saved pf_demo_final.gif")
