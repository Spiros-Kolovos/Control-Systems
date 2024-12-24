import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle
from matplotlib.widgets import Slider
from control import tf, feedback, step_response, step_info

# Define the open-loop transfer function G(s)
numerator = np.polymul([8],[2, 1])
denominator = np.polymul([1, 5], np.polymul([1, 2], [1, 3]))
G = tf(numerator, denominator)

# Set specifications
wn = 4
zeta = 0.5

# Initial PI controller gains
initial_Kp = 1.0  # Proportional gain
initial_Ki = 0.5  # Integral gain

# Time vector for step response
time = np.linspace(0, 20, 500)

# Define a function to calculate performance metrics
def calculate_metrics(TF):
    
    metrics = step_info(TF)
    # Overshoot percentage
    overshoot = metrics['Overshoot']

    # Settling time (2% criterion)
    settling_time = metrics['SettlingTime']

    # Steady-state error
    steady_state_error = 1.0 - metrics['SteadyStateValue']

    return overshoot, settling_time, steady_state_error

# Define a function to calculate root locus
def calculate_root_locus(poles_array, zeros_array, specification_wn = wn, specification_zeta = zeta):
    
    n = len(poles_array)
    m = len(zeros_array)
    if n > m:  # Asymptotes only exist if n > m
        # Calculate centroid
        centroid = (sum(poles_array) - sum(zeros_array)) / (n - m)

        # Calculate asymptotic angles
        angles = [180 * (2 * lamda + 1) / (n - m) for lamda in range(n - m)]
        
        # Plot new asymptotic lines
        for angle in angles:
            real_part = centroid + 20 * np.cos(np.radians(angle))
            imag_part = 20 * np.sin(np.radians(angle))
            line, = root_ax.plot([centroid, real_part], [0, imag_part], 'b--', label='Asymptote')
            asymptote_lines.append(line)
            
    asymptote_info = (
    f"Asymptotes:\n" +
    "\n".join([f"x = {centroid.real:.2f} ± {angle:.1f}°" for angle in angles])
    if n > m else "Asymptotes:\nNone"
    )
    
    # return the desired poles real part and imginary part
    real_part = -specification_wn * specification_zeta
    imag_part = specification_wn * np.sqrt(1 - specification_zeta ** 2)
    
    return asymptote_info, real_part, imag_part

# Define a function to update plots dynamically
def update(val):
    global legend
    Kp = kp_slider.val  # Get current Kp slider value
    Ki = ki_slider.val  # Get current Ki slider value
    C = tf([Kp, Ki], [1, 0])  # PI controller: Kp + Ki/s
    T = feedback(C * G, 1)  # Closed-loop transfer function

    # Update step response
    t, y = step_response(T, time)
    step_line.set_ydata(y)
    step_ax.relim()
    step_ax.autoscale_view()

    # Calculate and display performance metrics
    overshoot, settling_time, steady_state_error = calculate_metrics(TF=T)
    metrics_text.set_text(
        f"Overshoot: {overshoot:.2f}%\n"
        f"Settling Time: {settling_time:.2f}s\n"
        f"Steady-State Error: {steady_state_error:.2e}"
    )

    # Update poles and zeros
    poles = np.roots(T.den[0][0])
    zeros = np.roots(T.num[0][0])
    pole_scatter.set_offsets(np.column_stack((poles.real, poles.imag)))
    
    # Clear existing zero circles (if any) and re-plot all zeros
    dynamic_zero_circles = [artist for artist in root_ax.patches if artist not in desired_poles_circles and artist is not pd_zero_circle]
    for circle in dynamic_zero_circles:
        root_ax.patches.remove(circle)
    
    # Plot all zeros as no-filled black circles
    for zero in zeros:
        zero_circle = Circle((zero.real, zero.imag), radius=0.2, fill=False, color='black', lw=3)
        root_ax.add_patch(zero_circle)

    # Update zero position for PD controller
    if Kp != 0:  # Avoid division by zero
        pd_zero = -Ki / Kp
        pd_zero_circle.center = (pd_zero, 0)
        pd_zero_circle.set_visible(True)
    else:
        pd_zero_circle.set_visible(False)
        
    # Recalculate and update asymptotes
    for line in asymptote_lines:
        line.remove()  # Clear old asymptotes
    asymptote_lines.clear()

    #calculate root locus
    asymptote_info, real_part, imag_part = calculate_root_locus(poles, zeros)
            
    roots_text.set_text(
    f"Poles:\n{', '.join([f'{pole.real:.2f}+{pole.imag:.2f}j' for pole in poles[:2]])}"
    + (f",\n{', '.join([f'{pole.real:.2f}+{pole.imag:.2f}j' for pole in poles[2:]])}" if len(poles) > 2 else "") + "\n\n"
    f"Zeros:\n{', '.join([f'{zero.real:.2f}+{zero.imag:.2f}j' for zero in zeros])}\n\n"
    f"{asymptote_info}\n\n"
    f"Desired Poles: {real_part:.2f}±{imag_part:.2f}j"
    )

    # Update legend
    legend.remove()  # Remove old legend
    legend = step_ax.legend([step_line], [f"Kp={Kp:.1f}, Ki={Ki:.1f}"])  # Add updated legend

    # Refresh plots
    fig.canvas.draw_idle()

# Set up figure
fig, (step_ax, root_ax) = plt.subplots(1, 2, figsize=(12, 6), gridspec_kw={'width_ratios': [1, 1]})

# Step response plot
C = tf([initial_Kp, initial_Ki], [1, 0])  # Initial controller
T = feedback(C * G, 1)
t, y = step_response(T, time)
step_line, = step_ax.plot(t, y, label=f'Kp={initial_Kp:.1f}, Ki={initial_Ki:.1f}')
legend = step_ax.legend()  # Store initial legend
step_ax.set_title("Step Response")
step_ax.set_xlabel("Time (s)")
step_ax.set_ylabel("Response")
step_ax.grid()

# Add performance metrics as text box
overshoot, settling_time, steady_state_error = calculate_metrics(TF=T)
metrics_text = step_ax.text(
    0.55, 0.05,  # Position in axes coordinates
    f"Overshoot: {overshoot:.2f}%\n"
    f"Settling Time: {settling_time:.2f}s\n"
    f"Steady-State Error: {steady_state_error:.2e}",
    transform=step_ax.transAxes,
    fontsize=10,
    verticalalignment='bottom',
    horizontalalignment="left",
    bbox=dict(boxstyle="round", facecolor="white", edgecolor="black")
)

# Pole-zero plot
poles = np.roots(T.den[0][0])
zeros = np.roots(T.num[0][0])

# Create the initial zero for the PI controller (black circle, no fill)
initial_pd_zero = -initial_Ki / initial_Kp if initial_Kp != 0 else 0
pd_zero_circle = Circle((initial_pd_zero, 0), radius=0.2, fill=False, color='black', lw=3)
root_ax.add_patch(pd_zero_circle)

pole_scatter = root_ax.scatter(poles.real, poles.imag, color='red', marker='x', label='Actual Poles')
for zero in zeros:
    zero_circle = Circle((zero.real, zero.imag), radius=0.2, fill=False, color='black', lw=3)
    root_ax.add_patch(zero_circle)

# Initialize asymptotes
asymptote_lines = []

# calculate root locus for the first time
asymptote_info, real_part, imag_part = calculate_root_locus(poles, zeros)

# calculate desired poles
desired_poles = [complex(real_part, imag_part), complex(real_part, -imag_part)]
desired_poles_circles = []
for pole in desired_poles:
    circle = Circle((np.real(pole), np.imag(pole)), radius=0.2, fill=False, color='green', lw=3)
    root_ax.add_patch(circle)
    desired_poles_circles.append(circle)
    
# Define custom legend handles
legend_handles = [
    Line2D([0], [0], color='red', marker='x', linestyle='None', label='Actual Poles'),
    Line2D([0], [0], color='black', marker='o', linestyle='None', label='Zeros', markerfacecolor='none'),
    Line2D([0], [0], color='b', linestyle='--', label='Asymptotes'),
    Line2D([0], [0], color='green', marker='o', linestyle='None', label='Desired Poles', markerfacecolor='none')
]

roots_text = root_ax.text(
    0.02, 0.02,  # Position in axes coordinates
    f"Poles:\n{', '.join([f'{pole.real:.2f}+{pole.imag:.2f}j' for pole in poles])}\n\n"
    f"Zeros:\n{', '.join([f'{zero.real:.2f}+{zero.imag:.2f}j' for zero in zeros])}\n\n"
    f"{asymptote_info}\n\n"
    f"Desired Poles: {real_part:.2f}±{imag_part:.2f}j",
    transform=root_ax.transAxes,
    fontsize=10,
    verticalalignment='bottom',
    horizontalalignment="left",
    bbox=dict(boxstyle="round", facecolor="white", edgecolor="black")
)

# Set plot limits
root_ax.set_xlim(-10, 1)
root_ax.set_ylim(-15, 15)
root_ax.set_title("Poles and Zeros in the Real-Imaginary Plane")
root_ax.set_xlabel("Real")
root_ax.set_ylabel("Imaginary")
root_ax.legend(handles = legend_handles, loc = 'upper left', fontsize=10, frameon=True, fancybox=True, shadow=True)
root_ax.grid()

# Add sliders for Kp and Ki
kp_slider_ax = plt.axes([0.25, 0.01, 0.5, 0.03], facecolor='lightgoldenrodyellow')
ki_slider_ax = plt.axes([0.25, 0.05, 0.5, 0.03], facecolor='lightgoldenrodyellow')
kp_slider = Slider(kp_slider_ax, 'Kp', 0.1, 70.0, valinit=initial_Kp, valstep=0.1)
ki_slider = Slider(ki_slider_ax, 'Ki', 0.1, 70.0, valinit=initial_Ki, valstep=0.1)

# Attach the update function to the sliders
kp_slider.on_changed(update)
ki_slider.on_changed(update)

plt.tight_layout()
plt.show()
