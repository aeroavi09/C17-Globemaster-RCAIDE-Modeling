#Boeing C-17 Globemaster III migration

import RCAIDE
print("RCAIDE version: " + RCAIDE.__version__)

#imports

# python imports
import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt
import os
from pathlib import Path
import sys
sys.path.insert(0,(os.path.dirname(os.getcwd())))

print(sys.path.insert(0,(os.path.dirname(os.getcwd()))))

import RCAIDE
from RCAIDE.Framework.Core                                     import Units

from RCAIDE.Library.Methods.Geometry.Planform                  import segment_properties
from RCAIDE.Library.Plots import *

from RCAIDE.Framework.External_Interfaces.OpenVSP import get_vsp_measurements
from RCAIDE.Framework.External_Interfaces.OpenVSP import export_vsp_vehicle
from RCAIDE.Library.Methods.Geometry.Planform     import wing_segmented_planform
from RCAIDE.Framework.External_Interfaces.OpenVSP import write_vsp_mesh

from RCAIDE.Framework.External_Interfaces.OpenVSP.vsp_rotor     import write_vsp_rotor_bem
from RCAIDE.Framework.External_Interfaces.OpenVSP.vsp_fuselage  import write_vsp_fuselage
from RCAIDE.Framework.External_Interfaces.OpenVSP.vsp_wing      import write_vsp_wing
from RCAIDE.Framework.External_Interfaces.OpenVSP.vsp_nacelle   import write_vsp_nacelle 


from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan          import design_turbofan
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.compute_aircraft_moment_of_inertia import compute_aircraft_moment_of_inertia
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity              import compute_vehicle_center_of_gravity

from copy import deepcopy

import matplotlib.pyplot as plt
import numpy as np
import os
import sys
sys.path.insert(0,(os.path.dirname(os.getcwd())))
import  pickle


plane_name = 'C17_Globemaster_III'

def main():
    
    # vehicle data
    vehicle  = vehicle_setup() 
    
    # Set up vehicle configs
    configs  = configs_setup(vehicle)
    
    # create analyses
    analyses = analyses_setup(configs)
    
    # mission analyses 
    mission = mission_setup(analyses)
    
    # create mission instances (for multiple types of missions)
    missions = missions_setup(mission) 
    print("Segments in mission:")
    for seg in mission.segments:
        print(" -", seg.tag)
    # mission analysis 
    print("MAC:", vehicle.wings.main_wing.chords.mean_aerodynamic)
    print("CG:", vehicle.mass_properties.center_of_gravity)
    print("Takeoff mass:", vehicle.mass_properties.takeoff)
    results = missions.base_mission.evaluate()  
    
    # plot the results
    plot_mission(results) 
    
    # plot vehicle 
    #plot_3d_vehicle(vehicle) 
                    
    export_vsp_vehicle(vehicle, plane_name)
    
    return

def vehicle_setup():
    
    vehicle = RCAIDE.Vehicle()
    vehicle.tag = plane_name
    
    
    #this is the fuel and mass stuff
    vehicle.reference_area                                  = 3800 * Units.feet**2
    vehicle.mass_properties.max_takeoff                     = 585000* Units.pounds
    vehicle.mass_properties.takeoff                         = 586000* Units.pounds
    vehicle.mass_properties.operating_empty                 = 282500* Units.pounds
    vehicle.mass_properties.max_zero_fuel                   = 282500 * Units.pounds
    #vehicle.mass_properties.max_fuel                       = (180000+282500) * Units.pounds
    vehicle.mass_properties.cargo                           = 10000  * Units.pounds #170900lb max for c17
    vehicle.mass_properties.center_of_gravity               = [[65 * Units.feet,0, -1 * Units.feet, 0]]
    vehicle.passengers                                      = 134

    vehicle.flight_envelope.ultimate_load                   = 1.85
    vehicle.flight_envelope.positive_limit_load             = 1.40
    vehicle.flight_envelope.design_cruise_altitude          = 28000 * Units.ft
    vehicle.flight_envelope.design_range                    = 2400  * Units.nmi
    #vehicle.flight_envelope.design_dynamic_pressure        = 0.5*1.225*(254.722**2.) #Max q
    vehicle.flight_envelope.design_mach_number              = 0.76
    
    # FLOPS required system/accessories inputs
    vehicle.systems.accessories                             = 'long-range'   # options: 'short-range', 'long-range', 'business', 'cargo'
    vehicle.systems.electrical                              = 'basic'
    vehicle.systems.control                                 = 'fully-powered'
    vehicle.systems.avionics_weight                         = 1000 * Units.pounds
    vehicle.systems.hydraulics                              = 'medium'
    vehicle.systems.life_support                            = 'baseline'
    vehicle.systems.APU_weight                              = 1200 * Units.pounds
    
    
    #landing gear
    main_gear                                               = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear()
    main_gear.tire_diameter                                 = 3.81 * Units.feet
    main_gear.strut_length                                  = 3.175 * Units.feet
    main_gear.units                                         = 4    # Number of main landing gear
    main_gear.wheels                                        = 2    # Number of wheels on the main landing gear
    vehicle.append_component(main_gear)
    
    nose_gear = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()
    nose_gear.tire_diameter                                 = 3.175 * Units.feet
    nose_gear.units                                         = 1    # Number of nose landing gear
    nose_gear.wheels                                        = 2    # Number of wheels on the nose landing gear
    nose_gear.strut_length                                  = 4.445 * Units.feet
    vehicle.append_component(nose_gear)
    
    
    # main wing 
    
    wing = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                                                = 'main_wing' 
    wing.sweeps.quarter_chord                               = 25* Units.deg
    wing.thickness_to_chord                                 = 4.45/38.1
    wing.areas.reference                                    = 3800 * Units.feet**2
    wing.spans.projected                                    = 169 * Units.feet + 10 *Units.inches
    wing.chords.root                                        = 38.1 * Units.feet
    wing.chords.tip                                         = 2.54 * Units.feet
    wing.taper                                              = wing.chords.tip/wing.chords.root 
    wing.aspect_ratio                                       = wing.spans.projected **2 / wing.areas.reference 
    wing.twists.root                                        = 0.0 * Units.degrees
    wing.twists.tip                                         = 0 * Units.degrees
    wing.areas.wetted                                       = 2.0 * wing.areas.reference
    wing.areas.exposed                                      = 0.9 * wing.areas.wetted
    wing.areas.affected                                     = 0.9 * wing.areas.wetted    
    wing.chords.mean_aerodynamic                            = 20.6 * Units.feet  
    wing.twists.root                                        = 0.0 * Units.degrees
    wing.origin                                             = [[45.72 * Units.feet, 0 * Units.feet, 8.0655 * Units.feet]] #x,y,z
    wing.vertical                                           = False
    wing.symmetric                                          = True
    wing.high_lift                                          = True
    wing.dynamic_pressure_ratio                             = 1.0
    airfoil                                                 = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                                             = 'sc20714.dat'
    airfoil.coordinate_file                                 = './Airfoils/sc20714.dat.txt'
    wing.aerodynamic_center                                 =[0, 0, 0]
    # wing segments ------------
    
    #body
    body = RCAIDE.Library.Components.Wings.Segments.Segment()
    body.tag                                                = 'body' 
    body.percent_span_location                              = 0
    body.twist                                              = 0.0 * Units.deg
    body.root_chord_percent                                 = 1.0
    body.thickness_to_chord                                 = 4.45/38.1
    body.dihedral_outboard                                  = 0 * Units.degrees
    body.sweeps.quarter_chord                               = 0 * Units.degrees
    airfoil = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file = './Airfoils/sc20714.dat.txt'
    body.append_airfoil(airfoil)
    wing.append_segment(body) 
    
    #root
    root = RCAIDE.Library.Components.Wings.Segments.Segment()
    root.tag                                                = 'root' 
    root.percent_span_location                              = 0.12
    root.twist                                              = 0.0 * Units.deg
    root.root_chord_percent                                 = 1.0
    root.thickness_to_chord                                 = 4.45/38.1
    root.dihedral_outboard                                  = -3 * Units.degrees
    root.sweeps.quarter_chord                               = 25 * Units.degrees
    airfoil = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file = './Airfoils/sc20714.dat.txt'
    root.append_airfoil(airfoil)
    wing.append_segment(root) 
    
    #segment1
    segment1 = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment1.tag                                            = 'segment1' 
    segment1.percent_span_location                          = 0.47868080094 *2
    segment1.twist                                          = 0.0 * Units.deg
    segment1.root_chord_percent                             = 8.89/38.1
    segment1.thickness_to_chord                             = 1.905/8.89
    segment1.dihedral_outboard                              = 74 * Units.degrees
    segment1.sweeps.quarter_chord                           = 25 * Units.degrees
    airfoil = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file = './Airfoils/sc20714.dat.txt'
    segment1.append_airfoil(airfoil)
    wing.append_segment(segment1) 
        

    
    #winglet end
    tip = RCAIDE.Library.Components.Wings.Segments.Segment()
    tip.tag                                                 = 'tip' 
    tip.percent_span_location                               = 1.0
    tip.twist                                               = 0 * Units.deg
    tip.root_chord_percent                                  = wing.chords.tip/wing.chords.root
    tip.thickness_to_chord                                  = 1.27/2.54
    tip.dihedral_outboard                                   = 74 * Units.degrees
    tip.sweeps.quarter_chord                                = 0 * Units.degrees
    wing.append_segment(tip)

    
    # Fill out more segment properties automatically
    wing = segment_properties(wing)
    
    print(f"The Mean Aerodynamic Chord (MAC) is: {wing.chords.mean_aerodynamic / Units.feet} feet")
    
    # add to vehicle
    vehicle.append_component(wing)
    
    print("Wing AR:", wing.aspect_ratio)
    print("Wing area:", wing.areas.reference)
    print("Wing span:", wing.spans.projected)

    # add to vehicle
    
    # control surfaces --------------------
    spoiler = RCAIDE.Library.Components.Wings.Control_Surfaces.Spoiler()
    spoiler.tag                                             = 'spoiler'
    spoiler.span_fraction_start                             = 12.*2/169.83
    spoiler.span_fraction_end                               = 59.69*2/169.83
    spoiler.deflection                                      = 50 * Units.degrees
    spoiler.chord_fraction                                  = 0.17857142857
    wing.append_control_surface(spoiler)
    
    slat = RCAIDE.Library.Components.Wings.Control_Surfaces.Slat()
    slat.tag                                                = 'slat'            # ← must be exactly 'slat', so only 1 total
    slat.span_fraction_start                                = 12.5*2/169.83     # start of  slat1
    slat.span_fraction_end                                  = 42*2/169.83       # end of  slat2
    slat.deflection                                         = 0.0 * Units.degrees
    slat.chord_fraction                                     = 0.08928571428
    wing.append_control_surface(slat)
    
    flap = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    flap.tag                                                = 'flap'
    flap.span_fraction_start                                = 12.7*2/169.83
    flap.span_fraction_end                                  = 59.69*2/169.83
    flap.deflection                                         = 50 * Units.degrees
    flap.configuration_type                                 = 'multi_slotted'
    flap.chord_fraction                                     = 0.30
    wing.append_control_surface(flap)
    
    aileron = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                                             = 'aileron'
    aileron.span_fraction_start                             = 60.325*2/169.83
    aileron.span_fraction_end                               = 0.95
    aileron.deflection                                      = 25 * Units.degrees
    aileron.chord_fraction                                  = 0.16
    wing.append_control_surface(aileron)
    
    
    
    # vertical stabilizer  
    wing = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag                                                = 'vertical_stabilizer' 
    wing.sweeps.quarter_chord                               = 40.9 * Units.deg
    wing.thickness_to_chord                                 = 3.81/23.1683
    wing.areas.reference                                    = 632.88333 * Units.feet**2
    wing.spans.projected                                    = 24.916 * Units.feet
    wing.chords.root                                        = 24.29666 * Units.feet
    wing.chords.tip                                         = wing.chords.root+1*Units.feet
    wing.taper                                              = wing.chords.tip/wing.chords.root 
    wing.aspect_ratio                                       = wing.spans.projected **2 / wing.areas.reference 
    wing.twists.root                                        = 0.0 * Units.degrees
    wing.twists.tip                                         = 0.0 * Units.degrees
    wing.areas.wetted                                       = 2.0 * wing.areas.reference
    wing.areas.exposed                                      = 0.8 * wing.areas.wetted
    wing.areas.affected                                     = 0.6 * wing.areas.wetted    
    wing.twists.root                                        = 0.0 * Units.degrees
    wing.origin                                             = [[118.2 * Units.feet, 0 * Units.feet, 10.50666 * Units.feet]] #x,y,z
    wing.chords.mean_aerodynamic                            = 16.20 * Units.feet
    wing.vertical                                           = True
    wing.symmetric                                          = False
    wing.t_tail                                             = True
    wing.high_lift                                          = False
    wing.dynamic_pressure_ratio                             = 1.0
    wing.aerodynamic_center                                 =[0, 0, 0]
    
    #root
    root = RCAIDE.Library.Components.Wings.Segments.Segment()
    root.tag                                                = 'root' 
    root.percent_span_location                              = 0
    root.twist                                              = 0.0 * Units.deg
    root.root_chord_percent                                 = 1.0
    root.thickness_to_chord                                 = 3.81/23.1683
    root.sweeps.quarter_chord                               = 40.9 * Units.degrees
    wing.append_segment(root) 
    

    
    #tip
    tip = RCAIDE.Library.Components.Wings.Segments.Segment()
    tip.tag                                                 = 'tip' 
    tip.percent_span_location                               = 1
    tip.twist                                               = 0.0 * Units.deg
    tip.root_chord_percent                                  = 25.29666/24.29666
    tip.thickness_to_chord                                  = 3.92/25.4
    tip.sweeps.quarter_chord                                = 40.9 * Units.degrees
    wing.append_segment(tip) 
    
    wing = segment_properties(wing)
    vehicle.append_component(wing)
    
    rudder                                                  = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    rudder.tag                                              = 'rudder'
    rudder.span_fraction_start                              = .5/24.916
    rudder.span_fraction_end                                = 10.5*2/24.916
    rudder.deflection                                       = 25 * Units.degrees
    rudder.chord_fraction                                   = .33333
    wing.append_control_surface(rudder)
    
    
    # horizontal stabilizer  
    wing                                                    = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag                                                = 'horizontal_stabilizer' 
    wing.sweeps.quarter_chord                               = 27.47 * Units.deg
    wing.thickness_to_chord                                 = 2.574/16.51
    wing.areas.reference                                    = 383.064 * Units.feet**2
    wing.spans.projected                                    = 65 * Units.feet
    wing.chords.root                                        = 17.78 * Units.feet
    wing.chords.tip                                         = 7.62 * Units.feet
    wing.taper                                              = wing.chords.tip/wing.chords.root 
    wing.aspect_ratio                                       = wing.spans.projected **2 / wing.areas.reference 
    wing.twists.root                                        = 0.0 * Units.degrees
    wing.twists.tip                                         = 0.0 * Units.degrees
    wing.areas.wetted                                       = 2.0 * wing.areas.reference
    wing.areas.exposed                                      = 0.8 * wing.areas.wetted
    wing.areas.affected                                     = 0.6 * wing.areas.wetted    
    wing.twists.root                                        = 0.0 * Units.degrees
    wing.origin                                             = [[141.1475 * Units.feet, .7 * Units.feet, 32.79 * Units.feet]] #x,y,z
    wing.chords.mean_aerodynamic                            = 13.35  * Units.feet
    wing.vertical                                           = False
    wing.symmetric                                          = True
    wing.t_tail                                             = True
    wing.high_lift                                          = False
    wing.dynamic_pressure_ratio                             = 1.0
    wing.aerodynamic_center                                 =[0, 0, 0]
    airfoil = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file = './Airfoils/sc20714.dat.txt'
    
    hs_root = RCAIDE.Library.Components.Wings.Segments.Segment()
    hs_root.tag                                             = 'root_segment'
    hs_root.percent_span_location                           = 0.0
    hs_root.twist                                           = 0. * Units.deg
    hs_root.root_chord_percent                              = 1.0
    hs_root.dihedral_outboard                               = 3.0 * Units.degrees
    hs_root.sweeps.quarter_chord                            = 27.47 * Units.degrees
    hs_root.thickness_to_chord                              = 2.574/16.51
    hs_root.append_airfoil(airfoil)
    wing.append_segment(hs_root)
    
    # tip segment
    hs_tip = RCAIDE.Library.Components.Wings.Segments.Segment()
    hs_tip.tag                                              = 'tip_segment'
    hs_tip.percent_span_location                            = 1.0
    hs_tip.twist                                            = 0. * Units.deg
    hs_tip.root_chord_percent                               = 7.62/17.78
    hs_tip.dihedral_outboard                                = 0.0 * Units.degrees
    hs_tip.sweeps.quarter_chord                             = 0.0 * Units.degrees
    hs_tip.thickness_to_chord                               = 2.574/16.51
    wing.append_segment(hs_tip)
    
    wing = segment_properties(wing)
    vehicle.append_component(wing)
    
    elevator = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
    elevator.tag                                            = 'elevator'
    elevator.span_fraction_start                            = .1
    elevator.span_fraction_end                              = .95
    elevator.deflection                                     = 25 * Units.degrees
    elevator.chord_fraction                                 = .39
    wing.append_control_surface(elevator)
    
    fuselage = RCAIDE.Library.Components.Fuselages.Tube_Fuselage()
    fuselage.tag = 'fuselage'
    fuselage.lengths.total                                  = 150.579 * Units.feet
    fuselage.lengths.nose                                   = 25.74 * Units.feet
    fuselage.lengths.tail                                   = 69.498 * Units.feet
    fuselage.fineness.nose                                  = 27.94/21.008
    fuselage.fineness.tail                                  = 21.008/69.498
    fuselage.heights.maximum                                = 10.296*2 * Units.feet
    fuselage.width                                          = 28.886 * Units.feet
    fuselage.effective_diameter                             = 10.794 * Units.feet   # this becomes radius_outer
    fuselage.areas.wetted                                   = 20000 * Units.feet**2
    fuselage.heights.at_quarter_length                      = 8.3655 * Units.meter
    fuselage.heights.at_three_quarters_length               = 10.296 * Units.meter
    fuselage.heights.at_wing_root_quarter_chord             = 8.3655 * Units.meter
    fuselage.lengths.fore_space                             = 7.0     * Units.meter
    fuselage.areas.side_projected                           = 3100.0  * Units.feet**2
    fuselage.areas.front_projected                          = 165.0   * Units.feet**2
    fuselage.differential_pressure                          = 5.0e4 * Units.pascal
    fuselage.lengths.aft_space                              = 8.5    * Units.meter
    fuselage.number_coach_seats                             = vehicle.passengers
    fuselage.seats_abreast                                  = 6
    fuselage.seat_pitch                                     = 1 * Units.meter
    
    
    # Segment
    segment = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                             = 'segment1'
    segment.percent_x_location                              = 0.012
    segment.percent_z_location                              = -0.0025
    segment.height                                          = 3.2175 *Units.feet
    segment.width                                           = 3.0175 *Units.feet
    fuselage.segments.append(segment)
    
    # Segment
    segment2 = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment2.tag                                            = 'segment2'
    segment2.percent_x_location                             = 0.02256
    segment2.percent_z_location                             = -0.0025
    segment2.height                                         = 7.6525 *Units.feet
    segment2.width                                          = 7.4525 *Units.feet
    fuselage.segments.append(segment2)
    
    # Segment
    segment3 = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment3.tag                                            = 'segment3'
    segment3.percent_x_location                             = 0.056
    segment3.percent_z_location                             = -0.0035
    segment3.height                                         = 11.87 *Units.feet
    segment3.width                                          = 11.55 *Units.feet
    fuselage.segments.append(segment3)
    
    # Segment
    segment4 = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment4.tag                                            = 'segment4'
    segment4.percent_x_location                             = 0.11607
    segment4.percent_z_location                             = 0.0025
    segment4.height                                         = 18.018 *Units.feet
    segment4.width                                          = 18.018 *Units.feet
    fuselage.segments.append(segment4)
 
    # Segment
    segment5 = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment5.tag                                            = 'segment5'
    segment5.percent_x_location                             = 0.1738
    segment5.percent_z_location                             = 0.0025
    segment5.height                                         = 20.592 *Units.feet
    segment5.width                                          = 22.68 *Units.feet
    fuselage.segments.append(segment5)
    
    # Segment
    segment6 = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment6.tag                                            = 'segment6'
    segment6.percent_x_location                             = 0.2783
    segment6.percent_z_location                             = 0.0025
    segment6.height                                         = 20.592 *Units.feet
    segment6.width                                          = 22.68 *Units.feet
    fuselage.segments.append(segment6)
        
    # Segment
    segment7 = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment7.tag                                            = 'segment7'
    segment7.percent_x_location                             = 0.3036
    segment7.percent_z_location                             = 0.0025
    segment7.height                                         = 20.592 *Units.feet
    segment7.width                                          = 28.886 *Units.feet
    fuselage.segments.append(segment7)
    
    # Segment
    segment8 = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment8.tag                                            = 'segment8'
    segment8.percent_x_location                             = 0.5482
    segment8.percent_z_location                             = 0.0025
    segment8.height                                         = 20.592 *Units.feet
    segment8.width                                          = 28.886 *Units.feet
    fuselage.segments.append(segment8)
    
    # Segment
    segment9 = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment9.tag                                            = 'segment9'
    segment9.percent_x_location                             = 0.6072
    segment9.percent_z_location                             = 0.008
    segment9.height                                         = 19.03 * Units.feet
    segment9.width                                          = 22.68 *Units.feet
    fuselage.segments.append(segment9)
    
    # Segment
    segment10 = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment10.tag                                           = 'segment10'
    segment10.percent_x_location                            = .7935
    segment10.percent_z_location                            = 0.027
    segment10.height                                        = 13.243 * Units.feet
    segment10.width                                         = 22.68 *Units.feet
    fuselage.segments.append(segment10)
    
    # Segment
    segment11 = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment11.tag                                           = 'segment11'
    segment11.percent_x_location                            = .959
    segment11.percent_z_location                            = 0.045
    segment11.height                                        = 6.565 * Units.feet
    segment11.width                                         = 8.3 *Units.feet
    fuselage.segments.append(segment11)
    
    # Segment
    segment12 = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment12.tag                                           = 'segment12'
    segment12.percent_x_location                            = .962
    segment12.percent_z_location                            = 0.04
    segment12.height                                        = 4.965 * Units.feet
    segment12.width                                         = 8.3 *Units.feet
    fuselage.segments.append(segment12)
    
    # tip
    tip = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    tip.tag                                                 = 'tip'
    tip.percent_x_location                                  = 1.0
    tip.percent_z_location                                  = .04
    tip.height                                              = 0 *Units.feet
    tip.width                                               = 0 *Units.feet
    fuselage.segments.append(tip)
    
    
    vehicle.append_component(fuselage)
    
    net = RCAIDE.Framework.Networks.Fuel()

    #-------------------------------------------------------------------------------------------------------------------------
    # Fuel Distrubition Line
    #-------------------------------------------------------------------------------------------------------------------------
    fuel_line = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()

    #------------------------------------------------------------------------------------------------------------------------------------
    # Propulsor: Starboard Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------
    turbofan = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan()
    turbofan.tag                                            = 'starboard_propulsor_1'
    turbofan.active_fuel_tanks                              = ['fuel_tank']
    turbofan.origin                                         = [[34 * Units.feet + 8 *Units.inches, 23.495 * Units.feet,1.93* Units.feet]]
    turbofan.engine_length                                  = 22.225
    turbofan.bypass_ratio                                   = 5.9 
    turbofan.design_altitude                                = 28000*Units.ft
    turbofan.design_mach_number                             = .76
    turbofan.design_thrust                                  = (179886.08212113462/4) * Units.N #40440* Units.lbf

    # fan
    fan                                                     = RCAIDE.Library.Components.Powertrain.Converters.Fan()
    fan.tag                                                 = 'fan'
    fan.polytropic_efficiency                               = 0.93
    fan.pressure_ratio                                      = 1.74
    turbofan.fan                                            = fan

    # working fluid
    turbofan.working_fluid                                  = RCAIDE.Library.Attributes.Gases.Air()
    ram                                                     = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                                 = 'ram'
    turbofan.ram                                            = ram

    # inlet nozzle
    inlet_nozzle = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                                        = 'inlet nozzle'
    inlet_nozzle.polytropic_efficiency                      = 0.98
    inlet_nozzle.pressure_ratio                             = 0.98
    turbofan.inlet_nozzle                                   = inlet_nozzle

    # low pressure compressor
    low_pressure_compressor = RCAIDE.Library.Components.Powertrain.Converters.Compressor()
    low_pressure_compressor.tag                             = 'lpc'
    low_pressure_compressor.polytropic_efficiency           = 0.91
    low_pressure_compressor.pressure_ratio                  = 1.9
    turbofan.low_pressure_compressor                        = low_pressure_compressor

    # high pressure compressor
    high_pressure_compressor = RCAIDE.Library.Components.Powertrain.Converters.Compressor()
    high_pressure_compressor.tag                            = 'hpc'
    high_pressure_compressor.polytropic_efficiency          = 0.91
    high_pressure_compressor.pressure_ratio                 = 10.0
    turbofan.high_pressure_compressor                       = high_pressure_compressor

    # low pressure turbine
    low_pressure_turbine = RCAIDE.Library.Components.Powertrain.Converters.Turbine()
    low_pressure_turbine.tag                                ='lpt'
    low_pressure_turbine.mechanical_efficiency              = 0.99
    low_pressure_turbine.polytropic_efficiency              = 0.93
    turbofan.low_pressure_turbine                           = low_pressure_turbine

    # high pressure turbine
    high_pressure_turbine = RCAIDE.Library.Components.Powertrain.Converters.Turbine()
    high_pressure_turbine.tag                               ='hpt'
    high_pressure_turbine.mechanical_efficiency             = 0.99
    high_pressure_turbine.polytropic_efficiency             = 0.93
    turbofan.high_pressure_turbine                          = high_pressure_turbine

    # combustor
    combustor = RCAIDE.Library.Components.Powertrain.Converters.Combustor()
    combustor.tag                                           = 'Comb'
    combustor.efficiency                                    = 0.99
    combustor.alphac                                        = 1.0
    combustor.turbine_inlet_temperature                     = 1500
    combustor.pressure_ratio                                = 0.95
    turbofan.combustor                                      = combustor
    combustor.fuel_data = RCAIDE.Library.Attributes.Propellants.Jet_A1()

    # core nozzle
    core_nozzle = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()
    core_nozzle.tag                                         = 'core nozzle'
    core_nozzle.polytropic_efficiency                       = 0.95
    core_nozzle.pressure_ratio                              = 0.99
    turbofan.core_nozzle                                    = core_nozzle

    # fan nozzle
    fan_nozzle = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()
    fan_nozzle.tag                                          = 'fan nozzle'
    fan_nozzle.polytropic_efficiency                        = 0.95
    fan_nozzle.pressure_ratio                               = 0.99
    turbofan.fan_nozzle                                     = fan_nozzle

    # design turbofan
    design_turbofan(turbofan)
    # append propulsor to distribution line


    # Nacelle
    nacelle = RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle()
    nacelle.diameter                                        = 8.89 * Units.feet
    nacelle.length                                          = 16.51 * Units.feet
    nacelle.tag                                             = 'nacelle_'
    nacelle.inlet_diameter                                  = 8.79 * Units.feet
    nacelle.origin                                          = [[34 * Units.feet + 8 *Units.inches, 23.495 * Units.feet,1.93 * Units.feet]]
    nacelle.areas.wetted                                    = 1.1*np.pi*nacelle.diameter*nacelle.length
    nacelle_airfoil = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    nacelle_airfoil.NACA_4_Series_code                      = '2410'
    nacelle.append_airfoil(nacelle_airfoil)
    turbofan.nacelle                                        = nacelle

    # append propulsor to network
    net.propulsors.append(turbofan)
    
    # copy turbofan
    turbofan_2                                              = deepcopy(turbofan)
    turbofan_2.active_fuel_tanks                            = ['fuel_tank']
    turbofan_2.tag                                          = 'starboard_propulsor_2'
    turbofan_2.origin                                       = [[47 * Units.feet + 10 *Units.inches, 48.26 * Units.feet,0.53 * Units.feet]]  # change origin
    turbofan_2.nacelle.origin                               = [[47 * Units.feet + 10 *Units.inches, 48.26 * Units.feet,0.53 * Units.feet]]  # change origin
    
    # append propulsor to network
    net.propulsors.append(turbofan_2)
    
    # copy turbofan
    turbofan_3                                              = deepcopy(turbofan)
    turbofan_3.active_fuel_tanks                            = ['fuel_tank']
    turbofan_3.tag                                          = 'port_propulsor_1'
    turbofan_3.origin                                       = [[34 * Units.feet + 8 *Units.inches, -23.495 * Units.feet,1.93 * Units.feet]]  # change origin
    turbofan_3.nacelle.origin                               = [[34 * Units.feet + 8 *Units.inches, -23.495 * Units.feet,1.93 * Units.feet]]  # change origin
    
    # append propulsor to network
    net.propulsors.append(turbofan_3)
    
    # copy turbofan
    turbofan_4                                              = deepcopy(turbofan)
    turbofan_4.active_fuel_tanks                            = ['fuel_tank']
    turbofan_4.tag                                          = 'port_propulsor_2'
    turbofan_4.origin                                       = [[47 * Units.feet + 10 *Units.inches, -48.26 * Units.feet,0.53 * Units.feet]]  # change origin
    turbofan_4.nacelle.origin                               = [[47 * Units.feet + 10 *Units.inches, -48.26 * Units.feet,0.53 * Units.feet]]  # change origin
    
    # append propulsor to network
    net.propulsors.append(turbofan_4)
    
    # fuel tank
    fuel_tank = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank()
    fuel_tank.origin                                        = vehicle.wings.main_wing.origin
    fuel_tank.fuel                                          = RCAIDE.Library.Attributes.Propellants.Jet_A1()
    fuel_tank.fuel.mass_properties.mass                     = 137600 * Units.pounds
    fuel_tank.fuel.origin                                   = vehicle.wings.main_wing.mass_properties.center_of_gravity
    fuel_tank.fuel.mass_properties.center_of_gravity        = vehicle.wings.main_wing.aerodynamic_center
    fuel_tank.volume                                        = fuel_tank.fuel.mass_properties.mass/fuel_tank.fuel.density
    fuel_line.fuel_tanks.append(fuel_tank)
    
    fuel_line.assigned_propulsors = [['starboard_propulsor_1', 'starboard_propulsor_2', 
                                   'port_propulsor_1', 'port_propulsor_2']]
    
    # Append fuel line to fuel line to network
    net.fuel_lines.append(fuel_line)
    
    # Append energy network to aircraft
    vehicle.append_energy_network(net)
    

    return vehicle

def configs_setup(vehicle):

    configs     = RCAIDE.Library.Components.Configs.Config.Container() 
    
    base_config = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base'  
    configs.append(base_config) 
    
    #   Cruise Configuration
    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'cruise'
    configs.append(config)
    
    #   Takeoff Configuration
    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'takeoff'
    
    # Control Surfaces
    # C-17 standard takeoff flaps are roughly 25 degrees
    config.wings['main_wing'].control_surfaces.flap.deflection  = 25. * Units.deg 
    config.wings['main_wing'].control_surfaces.slat.deflection = 25. * Units.deg
    
    #Propulsion
    # Ensure these tags match the names you gave engines in your propulsion setup
    for engine_tag in ['starboard_propulsor_1', 'starboard_propulsor_2', 'port_propulsor_1', 'port_propulsor_2']:
        config.networks.fuel.propulsors[engine_tag].fan.angular_velocity = 3500. * Units.rpm
    
    # preformance limits
    config.V2_VS_ratio = 1.20 # Standard safety margin for takeoff climb
    
    config.landing_gears.main_gear.gear_extended = True
    config.landing_gears.nose_gear.gear_extended = True
    
    configs.append(config)
    
    #   Cutback Configuration
    
    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'cutback'
    
    # 1. Control Surfaces
    # C-17 flaps move from ~25 deg (Takeoff) to ~20 deg for the initial climb 
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection = 20. * Units.deg
    
    # 2. Propulsion (Updating to the 4-engine tags)
    # 2780 RPM is roughly 80% of max N1, which is a perfect 'cutback' power setting
    cutback_rpm = 2780. * Units.rpm
    
    for engine_tag in ['starboard_propulsor_1', 'starboard_propulsor_2', 'port_propulsor_1', 'port_propulsor_2']:
        config.networks.fuel.propulsors[engine_tag].fan.angular_velocity = cutback_rpm
        
    configs.append(config)
    

    #   Landing Configuration
    
    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'landing'
    
    # control Surfances
    # Full landing flaps for C-17 are 50 degrees
    config.wings['main_wing'].control_surfaces.flap.deflection  = 50. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection = 25. * Units.deg
    
    # Propulsion
    # Keeping the RPM around 2400 for blown-wing lift stability
    landing_rpm = 2400. * Units.rpm
    for engine_tag in ['starboard_propulsor_1', 'starboard_propulsor_2', 'port_propulsor_1', 'port_propulsor_2']:
        config.networks.fuel.propulsors[engine_tag].fan.angular_velocity = landing_rpm
    
    # Landing Gear
    config.landing_gears.main_gear.gear_extended = True
    config.landing_gears.nose_gear.gear_extended = True
    
    # Approach Speed 
    # Vref/Vs of 1.23 is standard; for tactical STOL, this can drop even lower, 
    # but 1.23 is the safest for a general MDO mission profile.
    config.Vref_VS_ratio = 1.23
    
    configs.append(config)
    
    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'short_field_takeoff'
    
    # Control Surfaces
    # Increased flap deflection for maximum powered lift (STOL)
    config.wings['main_wing'].control_surfaces.flap.deflection  = 35. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection = 25. * Units.deg
    
    #  Propulsion
    for engine_tag in ['starboard_propulsor_1', 'starboard_propulsor_2', 'port_propulsor_1', 'port_propulsor_2']:
        config.networks.fuel.propulsors[engine_tag].fan.angular_velocity = 3500. * Units.rpm
    
    # Landing Gear
    # Extended for takeoff
    config.landing_gears.main_gear.gear_extended = True
    config.landing_gears.nose_gear.gear_extended = True
    
    # Performance Limits
    # Lower ratio for tactical/short-field obstacle clearance
    config.V2_VS_ratio = 1.15 
    
    configs.append(config)
    
    #   Reverse Thrust Configuration
    
    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'reverse_thrust'
    
    # Control Surfaces 
    # Flaps are usually retracted to 30 or 50 depending on if this is 
    # a tactical descent or a ground rollout.
    config.wings['main_wing'].control_surfaces.flap.deflection  = 30. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection = 25. * Units.deg
    
    # Deploy Spoilers/Speed Brakes to dump lift and increase drag
    config.wings['main_wing'].control_surfaces.spoiler.deflection = 60. * Units.deg
    
    # Propulsion
    # High RPM is needed to generate the reverse mass flow
    reverse_rpm = 3000. * Units.rpm 
    for engine_tag in ['starboard_propulsor_1', 'starboard_propulsor_2', 'port_propulsor_1', 'port_propulsor_2']:
        config.networks.fuel.propulsors[engine_tag].fan.angular_velocity = reverse_rpm
        

    #Landing Gear
    config.landing_gears.main_gear.gear_extended = True
    config.landing_gears.nose_gear.gear_extended = True
    
    configs.append(config)
    
    return configs

def base_analysis(vehicle):
    
    analyses = RCAIDE.Framework.Analyses.Vehicle()
 
    #  Weights
    weights                 = RCAIDE.Framework.Analyses.Weights.Conventional()
    weights.aircraft_type   = 'Transport'
    weights.vehicle = vehicle

    analyses.append(weights)

    #  Aerodynamics Analysis
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.vehicle = vehicle 
    aerodynamics.settings.number_of_spanwise_vortices       = 10
    aerodynamics.settings.number_of_chordwise_vortices      = 2
    analyses.append(aerodynamics)  
 
    #  Energy
    energy= RCAIDE.Framework.Analyses.Energy.Energy()
    energy.vehicle =  vehicle  
    analyses.append(energy)

    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   

    return analyses    

def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()
    for tag, config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

def mission_setup(analyses):   
    
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission' 
    
    Segments = RCAIDE.Framework.Mission.Segments
    base_segment = Segments.Segment()
    
    #takeoff
    segment = Segments.Ground.Takeoff(base_segment)
    segment.tag = "takeoff"
    segment.analyses.extend( analyses.takeoff )
    segment.velocity_start                                          = 0.* Units.knots
    segment.velocity_end                                            = 150.0 * Units['m/s']
    segment.friction_coefficient                                    = 0.04
    segment.altitude                                                = 0.0
    mission.append_segment(segment)
    
    #first climb
    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1"
    segment.analyses.extend( analyses.takeoff )
    segment.altitude_start                                          = 0.0   * Units.feet
    segment.altitude_end                                            = 10000   * Units.feet
    segment.air_speed                                               = 125.0 * Units['m/s']
    segment.climb_rate                                              = 6.0   * Units['m/s']

    segment.flight_dynamics.force_x                                 = True
    segment.flight_dynamics.force_z                                 = True

    segment.assigned_control_variables.throttle.active              = True
    segment.assigned_control_variables.throttle.assigned_propulsors = [['starboard_propulsor_1', 'starboard_propulsor_2','port_propulsor_1', 'port_propulsor_2']]
    segment.assigned_control_variables.body_angle.active            = True

    mission.append_segment(segment)
    
    #second climb
    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_2"
    segment.analyses.extend( analyses.takeoff )
    segment.altitude_start                                          = 10000   * Units.feet
    segment.altitude_end                                            = 28000   * Units.feet
    segment.air_speed                                               = 230.0 * Units['m/s']
    segment.climb_rate                                              = 9.0   * Units['m/s']

    segment.flight_dynamics.force_x                                 = True
    segment.flight_dynamics.force_z                                 = True

    segment.assigned_control_variables.throttle.active              = True
    segment.assigned_control_variables.throttle.assigned_propulsors = [['starboard_propulsor_1', 'starboard_propulsor_2','port_propulsor_1', 'port_propulsor_2']]
    segment.assigned_control_variables.body_angle.active            = True

    mission.append_segment(segment)
    
    #first cruise after climbing
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise_1"
    segment.analyses.extend( analyses.cruise )
    segment.altitude                                                = 28000   * Units.feet
    segment.air_speed                                               = 520 * Units['mph']
    segment.distance                                                = 1000 * Units.nmi
    
    segment.flight_dynamics.force_x                                 = True
    segment.flight_dynamics.force_z                                 = True
    
    segment.assigned_control_variables.throttle.active              = True
    segment.assigned_control_variables.throttle.assigned_propulsors = [['starboard_propulsor_1', 'starboard_propulsor_2','port_propulsor_1', 'port_propulsor_2']]
    segment.assigned_control_variables.body_angle.active            = True
    
    mission.append_segment(segment)
    
    #descent to 10000 feet
    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_1"
    segment.analyses.extend( analyses.cruise )
    segment.altitude_start                                          = 28000   * Units.feet
    segment.altitude_end                                            = 10000   * Units.feet
    segment.air_speed                                               = 400 * Units['mph']
    segment.descent_rate                                            = 4.5   * Units['m/s']
    
    segment.flight_dynamics.force_x                                 = True
    segment.flight_dynamics.force_z                                 = True
    
    segment.assigned_control_variables.throttle.active              = True
    segment.assigned_control_variables.throttle.assigned_propulsors = [['starboard_propulsor_1', 'starboard_propulsor_2','port_propulsor_1', 'port_propulsor_2']]
    segment.assigned_control_variables.body_angle.active            = True
    
    mission.append_segment(segment)
    
    #second cruise after descent
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise_2"
    segment.analyses.extend( analyses.cruise )
    segment.altitude                                                = 10000   * Units.feet
    segment.air_speed                                               = 300 * Units['mph']
    segment.distance                                                = 50 * Units.nmi
    
    segment.flight_dynamics.force_x                                 = True
    segment.flight_dynamics.force_z                                 = True
    
    segment.assigned_control_variables.throttle.active              = True
    segment.assigned_control_variables.throttle.assigned_propulsors = [['starboard_propulsor_1', 'starboard_propulsor_2','port_propulsor_1', 'port_propulsor_2']]
    segment.assigned_control_variables.body_angle.active            = True
    
    mission.append_segment(segment)
    
    #third cruise after descent, slow down before airdrop
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise_3"
    segment.analyses.extend( analyses.cruise )
    segment.altitude                                                = 10000   * Units.feet
    segment.air_speed                                               = 160 * Units['mph']
    segment.distance                                                = 50 * Units.nmi
    
    segment.flight_dynamics.force_x                                 = True
    segment.flight_dynamics.force_z                                 = True
    
    segment.assigned_control_variables.throttle.active              = True
    segment.assigned_control_variables.throttle.assigned_propulsors = [['starboard_propulsor_1', 'starboard_propulsor_2','port_propulsor_1', 'port_propulsor_2']]
    segment.assigned_control_variables.body_angle.active            = True
    
    mission.append_segment(segment)
    
    #third climb
    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_3"
    segment.analyses.extend( analyses.takeoff )
    segment.altitude_start                                          = 10000   * Units.feet
    segment.altitude_end                                            = 28000   * Units.feet
    segment.air_speed                                               = 400 * Units['mph']
    segment.climb_rate                                              = 9.0   * Units['m/s']

    segment.flight_dynamics.force_x                                 = True
    segment.flight_dynamics.force_z                                 = True

    segment.assigned_control_variables.throttle.active              = True
    segment.assigned_control_variables.throttle.assigned_propulsors = [['starboard_propulsor_1', 'starboard_propulsor_2','port_propulsor_1', 'port_propulsor_2']]
    segment.assigned_control_variables.body_angle.active            = True

    mission.append_segment(segment)
    
    #fourth cruise after ascent
    segment = Segments.Cruise.Constant_Mach_Constant_Altitude(base_segment)
    segment.tag = "cruise_4"
    segment.analyses.extend( analyses.cruise )
    segment.altitude                                                = 28000   * Units.feet
    segment.mach_number                                             = 0.76
    segment.distance                                                = 500 * Units.nmi
    
    segment.flight_dynamics.force_x                                 = True
    segment.flight_dynamics.force_z                                 = True
    
    segment.assigned_control_variables.throttle.active              = True
    segment.assigned_control_variables.throttle.assigned_propulsors = [['starboard_propulsor_1', 'starboard_propulsor_2','port_propulsor_1', 'port_propulsor_2']]
    segment.assigned_control_variables.body_angle.active            = True
    
    mission.append_segment(segment)
    
    #descent after cruising to land
    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_2"
    segment.analyses.extend( analyses.cruise )
    segment.altitude_start                                          = 28000   * Units.feet
    segment.altitude_end                                            = 0   * Units.feet
    segment.air_speed                                               = 400 * Units['mph']
    segment.descent_rate                                            = 3.5   * Units['m/s']
    
    segment.flight_dynamics.force_x                                 = True
    segment.flight_dynamics.force_z                                 = True
    
    segment.assigned_control_variables.throttle.active              = True
    segment.assigned_control_variables.throttle.assigned_propulsors = [['starboard_propulsor_1', 'starboard_propulsor_2','port_propulsor_1', 'port_propulsor_2']]
    segment.assigned_control_variables.body_angle.active            = True
    
    mission.append_segment(segment)
    
    #land
    
    segment = Segments.Ground.Landing(base_segment)
    segment.tag = "landing"

    segment.analyses.extend( analyses.reverse_thrust )
    segment.velocity_start                                          = 400 * Units['mph']
    segment.velocity_end                                            = 10 * Units.knots
    segment.friction_coefficient                                    = 0.4
    segment.altitude                                                = 0.0
    segment.assigned_control_variables.elapsed_time.active          = True
    #segment.assigned_control_variables.elapsed_time.initial_guess_values  = [[60.]]
    mission.append_segment(segment)

    return mission 

def missions_setup(mission): 
 
    missions = RCAIDE.Framework.Mission.Missions()
    
    # base mission 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions 

def plot_mission(results):  
    
    # Plot Flight Conditions
    plot_flight_conditions(results)

    # Plot Aerodynamic Forces
    plot_aerodynamic_forces(results)

    # Plot Aerodynamic Coefficients
    plot_aerodynamic_coefficients(results)

    # Drag Components
    plot_drag_components(results)

    # Plot Altitude, sfc, vehicle weight
    plot_altitude_sfc_weight(results)

    # Plot Velocities
    plot_aircraft_velocities(results)
    plt.show()
    #find the rcaide outputs needed for you
    
    return 


main()
