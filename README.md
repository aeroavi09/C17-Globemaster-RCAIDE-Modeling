# C17-Globemaster-RCAIDE-Modeling
This project implements a full aerodynamic, propulsion, and mission simulation of the Boeing C-17 Globemaster III strategic airlifter using the RCAIDE (formerly SUAVE) multidisciplinary design framework developed at Stanford/USC.

<img src="C-17 Plots/C-17 Left Iso.png" width="75%" height="75%">

Technical Highlights
<ul>
  <li>Full 3D wing geometry built from scratch using segmented planform definitions with spanwise dihedral, sweep, taper, and thickness-to-chord distributions matched to C-17 specifications</li>
  <li>Supercritical airfoil (SC20714) applied across all wing segments including winglets</li>
  <li>Multi-segment fuselage cross-section model with 13 stations capturing the C-17's distinctive upswept tail and cargo floor geometry</li>
  <li>Complete empennage model including T-tail horizontal stabilizer and vertical fin with segmented sweep transitions</li>
  <li>Full control surface suite: multi-slotted Fowler flaps, leading edge slats, spoilers, ailerons, rudder, and elevator with per-configuration deflection scheduling</li>
  <li>Four-engine F117-PW-100 turbofan powertrain model with full thermodynamic cycle: ram inlet → LPC → HPC → combustor → HPT → LPT → fan nozzle → core nozzle</li>
  <li>Bypass ratio, polytropic efficiencies, pressure ratios, and turbine inlet temperature all calibrated to published engine data</li>
  <li>Fuel system modeled with wing-integrated fuel tank, density-based volume calculation, and center-of-gravity tracking</li>
  <li>Multiple flight configurations: cruise, takeoff, landing, short-field takeoff, cutback, and reverse thrust — each with independent control surface and RPM settings</li>
  <li>Vortex Lattice Method aerodynamics with spanwise and chordwise panel discretization across all lifting surfaces simultaneously</li>
  <li>FLOPS-compatible weight estimation inputs: accessories, hydraulics, avionics, APU, life support, and control system classifications</li>
  <li>OpenVSP geometry export for external mesh generation</li>
</ul>

Performance plots can be found in <a src="C-17 Plots">C-17 Plots</a> folder
Flight plan: takeoff roll, climb to 10,000ft, climb to 28,000ft, cruise 1,000nmi, descend to 10,000ft, slow to airdrop speed, payload drop, climb back to 28,000ft, cruise 500nmi return, descend to landing

Measurements derived from Air Force manuals, case study presentations, simulation manuals

This model has 4 independently-originated turbofans, 6 distinct flight configurations, a segmented supercritical wing with winglets, a 13-station fuselage, a T-tail empennage, and a multi-leg mission with a mid-flight mass discontinuity. Every segment boundary requires exact state continuity or the solver diverges.

Personally, this was my favorite MDO project so far as I learned a lot about modeling turbofan engines while also incorperating complex configs unique to one of my favorite defense aircrafts.
