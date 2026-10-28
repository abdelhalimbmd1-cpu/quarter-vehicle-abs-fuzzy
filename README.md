# Quarter-Vehicle ABS Fuzzy Logic Controller Model

MATLAB/Simulink implementation of the quarter-vehicle Anti-lock Braking System (ABS) with Fuzzy Logic Control (FLC) described in:

> Geleta, Gopal, and Lemi (2024), *"Vehicle anti-lock brake system - dynamic modeling and simulation based on MATLAB Simulink and CarSim"*, Res. Eng. Struct. Mater. 10(2): 495-511.

---

## Prerequisites
- MATLAB R2025a (or compatible)
- Simulink
- Fuzzy Logic Toolbox
- Python with `numpy` & `matplotlib` (for standalone high-resolution figure export)

---

## Run Order

1. **`params_abs.m`**
   Loads all vehicle parameters, physical constants, and road adhesion curves ($\mu$-slip for dry, wet, and ice).
   ```matlab
   params_abs;
   ```

2. **`create_abs_fis.m`**
   Generates and exports the Mamdani Fuzzy Inference System (`abs_fuzzy.fis`) with 7 Gaussian membership functions per variable and the 19-rule base from Table 3.
   ```matlab
   create_abs_fis('abs_fuzzy.fis');
   ```

3. **`build_abs_model.m`**
   Programmatically constructs the Simulink model (`abs_fuzzy_model.slx`) matching the exact topology, block naming, and coordinates of Fig. 7.
   ```matlab
   build_abs_model('abs_fuzzy_model');
   ```

4. **`plot_abs_results.m`**
   Simulates the ABS model and generates the 7 subplots matching Fig. 8 (a–g), saving the output to `fig8_results.png`.
   ```matlab
   plot_abs_results('abs_fuzzy_model');
   ```

5. **`simulate_road_comparison.m` (Option 1: Multi-Road Comparison)**
   Simulates and compares braking on Dry Asphalt, Wet Asphalt, and Icy Road surfaces, saving `fig_option1_road_comparison.png`.
   ```matlab
   simulate_road_comparison('abs_fuzzy_model');
   ```

6. **`simulate_abs_vs_nonabs.m` (Option 2: ABS vs. Non-ABS Locked Wheel)**
   Simulates and benchmarks emergency braking with ABS vs. panic locked-wheel skidding, saving `fig_option2_abs_vs_nonabs.png`.
   ```matlab
   simulate_abs_vs_nonabs('abs_fuzzy_model');
   ```

7. **`generate_all_figures.py`**
   Direct standalone runner producing all high-resolution figures with pure white background and sharp black text:
   - `fig8_results.png`: 7-panel reproduction of Fig. 8 (a–g).
   - `fig_option1_road_comparison.png`: Dry vs. Wet vs. Ice comparison.
   - `fig_option2_abs_vs_nonabs.png`: ABS Active vs. Non-ABS wheel lock benchmark.
   - `fig_fuzzy_surface_3d.png`: 3D control surface of the Mamdani FLC (Fig. 5).
