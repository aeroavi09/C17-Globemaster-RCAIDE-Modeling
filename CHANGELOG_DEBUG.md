# C-17 RCAIDE debug log (2026-10-03)

Goal: cruise L/D was ~3.4 (real C-17 mid-teens), fuel burn was impossible, and takeoff/landing had negative lift and drag.
Model: `C17_Globemaster_III.py`, RCAIDE 1.2.1, conda env `rcaide1.2.1`. The original script is kept as `C17_Globemaster_III_ORIGINAL.py`.

**Result: cruise L/D went from 3.37 to 13.9. Fuel burned went from 170.8 t (with only 62.4 t in the tank) to 51.7 t (with 108.2 t in the tank).**

Every number below comes from a full mission run. "c1" means the middle control point of cruise_1 (28,000 ft, Mach 0.76).

## Before / after summary

| step | change | c1 CL | c1 CD | c1 CDp | c1 CDi | **c1 L/D** | c1 thrust kN | c1 fuel kg/s | mass lost t |
|---|---|---|---|---|---|---|---|---|---|
| baseline | (none) | 0.438 | 0.1299 | 0.0288 | 0.0737 | **3.37** | 614 | 10.5 | 170.8 |
| F1 | spoilers stowed in flight | 0.496 | 0.0731 | 0.0288 | 0.0331 | **6.79** | 347 | 5.93 | 97.8 |
| F2 | cruise flaps 0 deg | 0.496 | 0.0731 | 0.0288 | 0.0331 | **6.79** | 347 | 5.93 | 97.8 |
| F3 | Oswald e from Raymer | 0.532 | 0.0531 | 0.0288 | 0.0145 | **10.03** | 252 | 4.31 | 66.9 |
| F4 | fuselage wetted area | 0.528 | 0.0476 | 0.0238 | 0.0142 | **11.09** | 226 | 3.86 | 63.6 |
| F4b | nacelle/pylon drag reference area | 0.537 | 0.0385 | 0.0149 | 0.0147 | **13.94** | 183 | 3.13 | 50.0 |
| F5 | weights from data sheets | 0.500 | 0.0359 | 0.0149 | 0.0128 | **13.95** | 170 | 2.91 | 45.2 |
| F6 | real flight plan, speeds and payload drop | 0.478 | 0.0344 | 0.0149 | 0.0116 | **13.91** | 163 | 2.79 | 61.7 (51.7 fuel + 10 payload) |

Cruise_4 L/D went from 2.70 to 13.52. TSFC was about 0.60 lb/lbf/h throughout, which is reasonable for the F117, so the engine model was never the problem.

## Root causes and fixes, in the order they were fixed

### F1: spoilers deployed at 50 deg for the entire flight (the biggest error)
- **Cause:** the base vehicle set `spoiler.deflection = 50 deg`, and every config except `reverse_thrust` inherited it. RCAIDE's spoiler model (`spoiler_drag.py`, from NASA TN-D-8162) does `CL -= 0.0075 * deflection_deg`, which removes **0.375** from CL. It does that *after* induced drag has been computed from the full lift. So the wing really carried CL 0.81 in the induced-drag calculation while the plots showed 0.44.
- **Fix:** spoiler 0 deg on the base vehicle. `reverse_thrust` (landing rollout) keeps 60 deg. Source: the DeltaSim procedures disarm the spoilers after takeoff and arm them before landing.
- **Effect:** L/D 3.37 to 6.79.

### F2: flaps at 50 deg in the cruise config
- **Cause:** the base vehicle set `flap.deflection = 50 deg`, and `cruise` is a copy of base. This produced the "50 deg all flight" line in the control-surface plot.
- **Fix:** flap 0 deg on the base vehicle. Takeoff, landing and the other configs set their own flap angles.
- **Effect:** none on the numbers. The VLM flap term was not being applied in cruise segments (CL reconciles exactly as 1.14 x clean-wing surrogate - 0.375 spoiler). [uncertain] why. The config is now correct anyway.

### F3: VLM induced drag is wrong with the SC(2)-0714 camber
- **Evidence:**
  - With the cambered airfoil, the VLM gives a clean-wing CL0 of **1.17** (Mach 0.22) and **1.53** (Mach 0.76), which implies a zero-lift angle of about -10 deg.
  - Thin-airfoil theory on the same parsed camber line gives alpha_L0 = **-5.6 deg**, a 2D cl0 of 0.61, and a 3D CL0 of about **0.44**.
  - CDi has a 0.03-0.045 offset even at zero lift.
  - Flat-plate airfoils give CL0 = 0 and CDi proportional to CL^2, so the planform itself is fine.
  - The cambered result is mesh-dependent: going to RCAIDE's default 15x5 vortices changes the fitted e from 0.70 to 1.74. It isn't reliable at any mesh tested.
  - Before this fix, the solver's CDi was **7.4x** the hand value (0.0737 vs 0.0100 at e = 0.8).
- **Fix:** `aerodynamics.settings.oswald_efficiency_factor` set from Raymer, *Aircraft Design: A Conceptual Approach*, Eq. 12.48: e = 1.78(1 - 0.045 AR^0.68) - 0.64 = **0.822**. Eq. 12.48 applies because the main panel's leading-edge sweep is 29.6 deg (hand-computed from 25 deg quarter-chord sweep and the taper), and Raymer's swept-wing Eq. 12.49 is only for sweep above 30 deg. Sensitivity: Eq. 12.49 would give e = 0.608. The value is computed in the script from the wing's AR, not hard-coded.
- **Effect:** L/D 6.79 to 10.03. **Hand check at cruise_1 after F6:** CL 0.4777, so CL^2/(pi x 7.590 x 0.822) = **0.01164**, and the solver gives **0.01164** (0% difference). This matches by construction, because RCAIDE now uses this formula.

### F4: fuselage wetted area 2.2x too high
- **Cause:** hand-entered `fuselage.areas.wetted = 20000 ft^2`. Integrating the script's own 12 fuselage cross-sections (ellipse perimeters x slant length) gives **8,972 ft^2 (834 m^2)**. A Torenbeek-style body formula with D = 21.6 ft gives about 8,300 ft^2, which agrees.
- **Fix:** 8,972 ft^2.
- **Effect:** CDp 0.0288 to 0.0238, L/D 10.03 to 11.09.

### F4b: nacelle drag over-counted 7.4x (RCAIDE 1.2.1 library issue, fixed in the script)
- **Cause:** `parasite_drag_nacelle` normalizes nacelle CD by the frontal area pi*D^2/4. `parasite_total` then rescales it by the **lateral** area pi*D*L, which over-counts by 4L/D = 7.4x. It also stores `drag*drag/S` per nacelle, which is why the nacelles showed as 0 in the breakdown. The pylon CD is already referenced to S_ref but gets rescaled a second time.
- **Hand check:** 4 nacelles x FF 1.188 x cf 0.0022 x S_wet 47.1 m^2 / 353 m^2 = **0.0014**. RCAIDE was adding **0.0103**.
- **Fix:** `parasite_total_consistent_reference()` in the script is RCAIDE's function with the nacelle and pylon reference areas corrected. It's plugged in through `aerodynamics.process.compute.drag.parasite.total`. The library source is untouched.
- **Effect:** CDp 0.0238 to 0.0149. The per-component CDp now sums to the total (0.01489 vs 0.01491). Each nacelle is 0.00035, matching the hand value of 0.000348. L/D 11.09 to 13.94.

### F5: weights didn't add up
- **Cause:**
  - Takeoff mass was 586,000 lb, which is above the 585,000 lb MTOW.
  - Empty weight + fuel + cargo was only 195.1 t, so **70.7 t was unaccounted for**.
  - Cargo was 10,000 **lb**.
  - Max zero-fuel weight repeated the empty weight.
  - RCAIDE doesn't stop burning mass when the tank is empty, which is why 170.8 t was "burned" from a 62.4 t tank.
- **Fix:** values from the data sheets in this folder:
  - Empty weight 282,500 lb (DeltaSim "Operating Weight")
  - MZFW 447,400 lb (DeltaSim)
  - Cargo **10,000 kg**
  - Fuel = 35,546 US gal (C17 Manual capacity) x 804 kg/m^3 = **108.2 t**
  - Takeoff mass = empty + cargo + fuel = **246.3 t** (under MTOW)
- **Effect:** start mass 265.8 to 246.3 t, and fuel burned stays well inside the tank.

### F6: mission profile matched to the real flight plan
- **Flight plan:** takeoff, climb to 10,000 ft, climb to 28,000 ft, cruise 1,000 nmi, descend to 10,000 ft, slow to airdrop speed, drop the payload, climb back to 28,000 ft, cruise 500 nmi, descend and land.
- **Speeds and configs** (sources: DeltaSim flight manual procedures and limits, C17 Manual):
  - Climbs are clean (flaps and slats up after takeoff). Before this, they used the takeoff config with 25 deg flaps and slats.
  - Climb 250 KCAS to 10,000 ft, then 310 KCAS to the Mach 0.74 crossover at about 25,000 ft, then Mach 0.74 to 28,000 ft.
  - Cruise at Mach 0.76.
  - Descent at Mach 0.74 down to 25,000 ft, then 310 KCAS to 10,000 ft, then 250 KCAS below 10,000 ft.
  - New `airdrop` config: slats 25 deg and 1/2 flaps (25 deg), within DeltaSim's 280 / 250 KCAS limits.
  - New `slow_to_airdrop` deceleration from 250 to 150 KCAS at -0.5 m/s^2.
  - New `approach` segment from 1,500 ft on a 3 deg glide path in the landing config.
- **Not in the data sheets [uncertain]:** rotation 140 kt (was 292 kt), approach and touchdown 130 kt (touchdown was 348 kt), and the airdrop leg at 150 KCAS (was 139 kt TAS in a clean config, which needed CL 1.83).
- **Payload drop:** `drop_payload_then_initialize_weights()` wraps RCAIDE's own weight initializer and subtracts the cargo at the start of climb_3. Verified: cruise_3 ends at 211.0 t and climb_3 starts at 201.0 t.
- **Two more RCAIDE 1.2.1 issues hit along the way:**
  - `Descent.Constant_CAS_Constant_Rate` declares `calibrated_airspeed`, but its solver reads `calibrated_air_speed`. The script sets the latter.
  - `Descent.Linear_Mach_Constant_Rate` evaluates the speed of sound before setting altitude, so it flies Mach 0.74 x sea-level a = **Mach 0.82** at altitude. The script uses a constant-TAS descent instead (227.8 m/s, Mach 0.735-0.745).

## Acceptance criteria

- [x] **Cruise L/D 12-18:** cruise_1 13.91, cruise_4 13.52. Climb and descent legs are 11.1-15.5.
- [x] **Hand CDi within 10% of solver:** 0.01164 vs 0.01164 (by construction, see F3).
- [~] **No negative lift/drag/CL in takeoff and landing.** Drag is now positive everywhere: takeoff CD 0.122-0.206, landing CD 0.021-0.031. Before, it was negative. Lift is still negative, explained:
  - **Takeoff CL = -1.31 to -1.36.** On the ground alpha is held at exactly 0, and RCAIDE zeroes the alpha-dependent lift at alpha = 0 (`evaluate_VLM.py:114`). That leaves only the flap and slat increments. RCAIDE's flap surrogate is trained only at 0 and 10 deg, then linearly extrapolated, and it gives **negative** dCL (-0.048/deg at Mach 0.21). That's despite RCAIDE's own convention that positive deflection means trailing edge down (`deflect_control_surface.py:350`). The slat term is exactly 0. These are inside the library. [uncertain] why the sign is negative. The ground roll is otherwise physical: speed is monotonic from 0 to 72 m/s and it takes 54 s.
  - **Landing CL = -0.45** is exactly -0.0075 x 60 deg of ground spoilers on rollout. Dumping lift after touchdown is real C-17 behavior; the magnitude comes from RCAIDE's Croom (1976) correlation.
- [x] **Fuel burned at most ~110 t, final weight at least empty weight:** 51.7 t burned out of 108.2 t. Final 184.6 t is above empty + remaining payload (128.1 t).
- [x] **Drag components sum to total:** total CD = **1.1 x (CDp + CDi + CDc + CDm + CDspoiler)** in every segment (difference 0.0000). The two missing terms were RCAIDE's default `trim_drag_correction_factor = 1.1` and spoiler drag, which isn't plotted.

## The original 7 hypotheses

| # | hypothesis | verdict |
|---|---|---|
| 1 | CDi too high from span/AR/e/units | Symptom agreed. Geometry was fine (AR = b^2/S = 7.590 exactly; span and area match the C17 Manual). Causes were the spoiler (F1) and the cambered VLM (F3). |
| 2 | cruise_3 near stall | Agreed. CL 1.83 was required by physics at 139 kt TAS clean, and the VLM has no CLmax limit. Fixed in F6 (now CL 1.62 at 150 KCAS with slats and 1/2 flaps). |
| 3 | thrust/fuel follow from drag | Mostly agreed. Plus a separate weight-bookkeeping error (F5). |
| 4 | takeoff/landing sign errors or divide-by-q | Partly. Divide-by-q only affected the first point. The real causes were alpha = 0 on the ground, the flap surrogate sign, spoilers, and 292 kt / 348 kt ground speeds. See acceptance criteria. |
| 5 | negative cruise AoA from incidence or zero-lift angle | Agreed it's wrong, from the zero-lift angle: the VLM alpha_L0 is about -10 deg vs -5.6 deg from theory. **Still about -7 deg after the fixes. Not fixed.** It doesn't affect drag now (F3), but AoA plots aren't meaningful. No wing incidence is modeled. |
| 6 | 50 deg surface | Both the spoiler and the flap (F1, F2). |
| 7 | drag doesn't sum | Agreed. Not a bug: x1.1 trim factor plus spoiler drag. |

**XFOIL polars:** none are loaded in the C-17 model. The wing uses only the SC(2)-0714 coordinates (camber line). No SC(2)-0714 polar exists on disk. The polars that do exist are NACA 4412 (Re 5e4-1e6), SD6080 (Helios), and the NACA x412 camber sweep (Re 1e6), all from XFOIL at Mach 0. They aren't valid near cruise Mach 0.76 (transonic, with shocks on a supercritical section).

## Open items (not changed; geometry needs your OK)

- **Horizontal tail area:** the script has 383 ft^2 with a 65 ft span (AR 11). The C17 Manual lists **845 ft^2**, 65 ft span, AR 5.0.
- **`fuselage.effective_diameter` = 10.794 ft:** the script comment says it "becomes radius_outer", but RCAIDE uses it as a diameter in the fuselage form factor. The manual says 22.5 ft. That's about 5% of fuselage drag.
- **Negative cruise AoA** (hypothesis 5): to fix it, model wing incidence and/or get a better zero-lift angle (more chordwise panels didn't help).
- **Speeds marked [uncertain]:** rotation, approach and airdrop speeds weren't in the data sheets.
- **Possible upstream bug reports for RCAIDE:**
  - nacelle reference area in `parasite_total`
  - descent-CAS attribute name mismatch
  - `Linear_Mach` descent speed of sound
  - flap surrogate sign
