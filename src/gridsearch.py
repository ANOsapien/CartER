import numpy as np

# --- Grid Search Parameter Definition ---
# These ranges are slightly outside the previous ones to explore new areas.
p_p_range = np.linspace(450, 750, 5)  # Proportional for position (shifted lower, wider range)
p_d_range = np.linspace(3.0, 4.5, 4)  # Derivative for position (shifted higher, wider range)
a_p_range = np.linspace(-0.03, -0.07, 4) # Proportional for angle (shifted higher, slightly narrower)
a_d_range = np.linspace(-0.005, -0.015, 4) # Derivative for angle (shifted higher, slightly narrower)

# --- Generation Logic ---
pid_sets = []
for p_p in p_p_range:
    for p_d in p_d_range:
        for a_p in a_p_range:
            for a_d in a_d_range:
                # p_i is set relative to p_p, a_i is kept at 0
                p_i = p_p * 0.7 
                a_i = 0
                # Using .1f to keep the format clean and consistent
                param_str = f"{p_p:.1f},{p_i:.1f},{p_d:.2f},{a_p:.3f},{a_i},{a_d:.4f}"
                pid_sets.append(param_str)

# --- Print the results in a copy-paste friendly format ---
print("PID_PARAM_SETS = [")
for param in pid_sets:
    print(f'    "{param}",')
print("]")

print(f"\n# ✅ Generated {len(pid_sets)} total PID parameter sets.")