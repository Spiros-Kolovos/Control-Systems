import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.widgets import Slider
from control import tf, feedback, step_response

# Define the open-loop transfer function G(s)
numerator = [1]
denominator = np.polymul([1, 0], np.polymul([1, 1], [1, 3]))
G = tf(numerator, denominator)

# Initial gain Kp
initial_Kp = 1.0

# Time vector for step response
time = np.linspace(0, 10, 500)

# Define a function to update plots dynamically
def update(val):
    global legend  # Ensure global reference to legend
    Kp = slider.val  # Get current slider value
    C = tf([Kp], [1])  # Controller
    T = feedback(C * G, 1)  # Closed-loop transfer function

    # Update step response
    t, y = step_response(T, time)
    step_line.set_ydata(y)
    step_ax.relim()
    step_ax.autoscale_view()

    # Update poles
    poles = np.roots(np.polyadd(denominator, [Kp]))
    pole_scatter.set_offsets(np.column_stack((poles.real, poles.imag)))

    # Update legend to reflect the current Kp value
    legend.remove()  # Remove old legend
    legend = step_ax.legend([step_line], [f'Kp={Kp:.1f}'])  # Add updated legend

    # Refresh plots
    fig.canvas.draw_idle()

# Set up figure
fig, (step_ax, root_ax) = plt.subplots(1, 2, figsize=(12, 6), gridspec_kw={'width_ratios': [1, 1]})

# Step response plot
C = tf([initial_Kp], [1])  # Initial controller
T = feedback(C * G, 1)
t, y = step_response(T, time)
step_line, = step_ax.plot(t, y, label=f'Kp={initial_Kp:.1f}')
legend = step_ax.legend()  # Store initial legend reference
step_ax.set_title("Step Response")
step_ax.set_xlabel("Time (s)")
step_ax.set_ylabel("Response")
step_ax.grid()

# Pole plot
poles = np.roots(np.polyadd(denominator, [initial_Kp]))
pole_scatter = root_ax.scatter(poles.real, poles.imag, color='red', marker='x', label=f'Kp={initial_Kp:.1f}')

# Add asymptotic lines (blue, dashed)
root_ax.axhline(0, color='black', lw=0.5, linestyle='--')  # Imaginary axis
root_ax.axvline(0, color='black', lw=0.5, linestyle='--')  # Real axis

# Calculate and plot asymptotes
zeros = np.roots(numerator)

n = len(poles)
m = len(zeros)

# Calculate centroid
centroid = (sum(poles) - sum(zeros)) / (n - m)

# Calculate asymptotic angles
angles = [180 * (2 * lamda + 1) / (n - m) for lamda in range(n - m)]

# Plot the asymptotic lines
for angle in angles:
    # Asymptote line: starts at centroid and extends at the specified angle
    real_part = centroid + 5 * np.cos(np.radians(angle))
    imag_part = 5 * np.sin(np.radians(angle))
    root_ax.plot([centroid, real_part], [0, imag_part], 'b--', label='Asymptote')
    
# set specifications
wn = 4
zeta = 0.6

real_part = -wn * zeta
imag_part = wn * np.sqrt(1 - zeta ** 2)

# calculate desired poles
desired_poles = [complex(real_part, imag_part), complex(real_part, -imag_part)]

# Add the legend with LaTeX-style scientific notation for wn and zeta
wn_label = r"$\omega_n = {:.2f}$".format(wn)  # LaTeX for ωn
zeta_label = r"$\zeta = {:.2f}$".format(zeta)  # LaTeX for ζ

# Plot desired poles as green circles
for pole in desired_poles:
    circle = Circle((np.real(pole), np.imag(pole)), radius=0.05, fill=False, color='green', lw=2)
    root_ax.add_patch(circle)

# Set plot limits and update layout
root_ax.set_xlim(-10, 1)
root_ax.set_ylim(-10, 10)
root_ax.set_title("Poles in the Real-Imaginary Plane")
root_ax.set_xlabel("Real")
root_ax.set_ylabel("Imaginary")
root_ax.legend([f"{wn_label}, {zeta_label}"], loc = 'upper left', fontsize=10, frameon=True, fancybox=True, shadow=True)
root_ax.grid()

# Add slider
slider_ax = plt.axes([0.25, 0.01, 0.5, 0.03], facecolor='lightgoldenrodyellow')
slider = Slider(slider_ax, 'Kp', 0.1, 50.0, valinit=initial_Kp, valstep=0.1)

# Attach the update function to the slider
slider.on_changed(update)

plt.tight_layout()  # Adjust layout to avoid overlap
plt.show()
