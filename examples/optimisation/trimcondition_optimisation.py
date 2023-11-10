# --- Built-ins ---
from pathlib import Path
import os
import logging
import copy

# --- Internal ---
from src.utils.tools import print_results
from src.postprocessing.plots import all_plots, stackedplots_wing, stackedplots_prop
from src.integration.coupled_groups_optimisation import WingSlipstreamPropOptimisation
from examples.example_classes.PROWIM_classes import PROWIM_wingpropinfo

# --- External ---
import openmdao.api as om
import numpy as np


logging.getLogger('matplotlib.font_manager').disabled = True
BASE_DIR = Path(__file__).parents[0]

if __name__ == '__main__':
    # Start from 
    PROWIM_wingpropinfo.wing.empty_weight = 4 # to make T=D
    PROWIM_wingpropinfo.wing.CL0 = 0. # to make T=D
    # PROWIM_wingpropinfo.spanwise_discretisation_propeller = 21
    
    PROWIM_wingpropinfo.wing.twist = np.array([-0.57861226,  0.31019075,  1.98300295,  3.50895936,  5.08440816,
                                                6.27917302,  7.84406102,  7.02681601,  7.7773402 ,  6.33827094,
                                                5.10166953,  3.51365803,  1.99177001,  0.31332017, -0.57425951])
    PROWIM_wingpropinfo.wing.chord =  np.array([0.08333231, 0.08334073, 0.08331503, 0.08335375, 0.08328596,
                                                0.08350007, 0.0824684 , 0.09730153, 0.08240013, 0.08357395,
                                                0.08327046, 0.08334107, 0.08333046, 0.08333441, 0.08333284]) 
    PROWIM_wingpropinfo.wing.thickness = np.array([0.003, 0.003, 0.003, 0.003, 0.003, 0.003, 0.003, 0.003, 0.003,
                                                    0.003, 0.003, 0.003, 0.003, 0.003, 0.003])
                                                
    for iprop, _ in enumerate(PROWIM_wingpropinfo.propeller):
        PROWIM_wingpropinfo.propeller[iprop].rot_rate = 245.01782981
        PROWIM_wingpropinfo.propeller[iprop].twist = np.array([ 85.55251555, 83.64576076, 81.8520216,  80.09952685, 78.25649516, 77.8377415,
                                                                76.81006132, 75.58466854, 74.12662243, 72.71758514, 71.24466923, 69.71031773,
                                                                68.18307205, 66.63252791, 65.08761103, 63.55524488, 61.97682458, 60.31914345,
                                                                58.52826478, 56.19633768])
                                                                            
    # === Plotting ===
    # db_name = os.path.join(BASE_DIR, 'results', 'data_wingprop.db')
    # savepath = os.path.join(BASE_DIR, 'results', 'propwing_results')
    # stackedplots_prop(db_name=db_name,
    #                 wingpropinfo=PROWIM_wingpropinfo,
    #                 savedir=savepath)
    # stackedplots_wing(db_name=db_name,
    #             wingpropinfo=PROWIM_wingpropinfo,
    #             savedir=savepath)
    # quit()
    
    objective = {
                'HELIX_COUPLED.power_total':
                    {'scaler': 1/(112.60310745)}
                }

    design_vars = {
                    'DESIGNVARIABLES.rotor_0_rot_rate':
                        {'lb': 0,
                        'ub': 3000,
                        'scaler': 1./PROWIM_wingpropinfo.propeller[0].rot_rate},
                    'DESIGNVARIABLES.rotor_1_rot_rate':
                        {'lb': 0,
                        'ub': 3000,
                        'scaler': 1./PROWIM_wingpropinfo.propeller[1].rot_rate},
                    'DESIGNVARIABLES.rotor_0_twist':
                        {'lb': 0,
                        'ub': 90,
                        'scaler': 1./10},
                    'DESIGNVARIABLES.rotor_1_twist':
                        {'lb': 0,
                        'ub': 90,
                        'scaler': 1./10},
                    'PARAMETERS.alpha':
                        {'lb': -10,
                        'ub': 10,
                        'scaler': 1},
                    'DESIGNVARIABLES.twist':
                        {'lb': -10,
                        'ub': 8,
                        'scaler': 1},
                    'DESIGNVARIABLES.chord':
                        {'lb': 0,
                        'ub': 3,
                        'scaler': 1},
                    'OPENAEROSTRUCT.wing.thickness_cp':
                        {'lb': 3e-3,
                        'ub': 3e-3,
                        'scaler': 1/3e-3},
                    }

    constraints = {
                    'OPENAEROSTRUCT.AS_point_0.wing_perf.failure':
                        {'upper': 0.},
                    'OPENAEROSTRUCT.AS_point_0.total_perf.CL':
                        {'upper': 0.9},
                    'OPENAEROSTRUCT.AS_point_0.wing_perf.thickness_intersects':
                        {'upper': 0.},
                    'OPENAEROSTRUCT.AS_point_0.L_equals_W':
                        {'equals': 0.},
                    'CONSTRAINTS.thrust_equals_drag':
                        {'equals': 0.},
                    'OPENAEROSTRUCT.wing.structural_mass':
                        {'lower': 0.},
                    'OPENAEROSTRUCT.AS_point_0.total_perf.L':
                        {'lower': 0.},
                    'OPENAEROSTRUCT.AS_point_0.total_perf.D':
                        {'lower': 0.}
                    }

    prob = om.Problem()
    prob.model = WingSlipstreamPropOptimisation(WingPropInfo=PROWIM_wingpropinfo,
                                                objective=objective,
                                                constraints=constraints,
                                                design_vars=design_vars)

    # === Analysis ===
    prob.setup()
    prob.run_model()
    
    prob.run_model()
    prob.check_totals(
        compact_print=True,
        show_only_incorrect=True
    )
    quit()

        # Check derivatives  
    if False:
        prob.check_totals(  compact_print=True, show_only_incorrect=True,
                        form='central', step=1e-8, 
                        rel_err_tol=1e-3)
        # prob.check_partials(compact_print=True, show_only_incorrect=True, 
        #                             excludes=['*HELIX_0*', '*HELIX_1*'], 
        #                             form='central', step=1e-8)
        
        quit()

    print_results(design_vars=design_vars, constraints=constraints, objective=objective,
                  prob=prob, kind="Initial Analysis")
    
    # quit()

    # === Optimisation ===
    prob.driver = om.pyOptSparseDriver()
    prob.driver.options['optimizer'] = 'SNOPT'
    prob.driver.options['debug_print'] = ['desvars', 'nl_cons', 'objs']
    prob.driver.opt_settings = {
        "Major feasibility tolerance": 1.0e-5,
        "Major optimality tolerance": 1.0e-5,
        "Minor feasibility tolerance": 1.0e-5,
        "Verify level": -1,
        "Function precision": 1.0e-6,
        # "Major iterations limit": 1,
        "Nonderivative linesearch": None,
        "Print file": os.path.join(BASE_DIR, 'results', 'optimisation_print_wingprop_trim.out'),
        "Summary file": os.path.join(BASE_DIR, 'results', 'optimisation_summary_wingprop_trim.out')
    }
    
    # prob.check_totals(  compact_print=True, show_only_incorrect=True,
    #                     form='central', step=1e-8, 
    #                     rel_err_tol=1e-3)
    
        # Initialise recorder
    db_name = os.path.join(BASE_DIR, 'results', 'data_wingprop_trim.db')
    
    includes = ["OPENAEROSTRUCT.wing.geometry.twist",
                "OPENAEROSTRUCT.wing.geometry.chord",
                "OPENAEROSTRUCT.AS_point_0.wing_perf.Cl",
                "OPENAEROSTRUCT.AS_point_0.wing_perf.CDi",
                'OPENAEROSTRUCT.AS_point_0.total_perf.L',
                'OPENAEROSTRUCT.AS_point_0.total_perf.D',
                'RETHORST.velocity_distribution',
                'RETHORST.propeller_velocity',
                "HELIX_0.om_helix.rotorcomp_0_radii",
                "HELIX_0.om_helix.rotorcomp_0_velocity_distribution",
                "HELIX_0.om_helix.geodef_parametric_0_twist",
                "HELIX_0.om_helix.geodef_parametric_0_rot_rate",
                "PARAMETERS.wing_mesh",
                "PARAMETERS.wing_mesh_control_points",
                'OPENAEROSTRUCT.wing.structural_mass']
    
    for key in design_vars.keys():
        includes.extend(key)
    for key in design_vars.keys():
        includes.extend(key)
    for key in design_vars.keys():
        includes.extend(key)
    
    recorder = om.SqliteRecorder(db_name)
    prob.driver.add_recorder(recorder)
    prob.driver.add_recorder(recorder)
    # TODO: write code that checks whether the problem variable exists
    prob.driver.recording_options['includes'] = includes
    
    print('==========================================================')
    print('====================== Optimisation ======================')
    print('==========================================================')
    prob.setup()
    prob.run_driver()
    
    prob.cleanup() # close all recorders

    print_results(design_vars=design_vars, constraints=constraints, objective=objective,
                  prob=prob, kind="Results")
    
    # === Plotting ===
    savepath = os.path.join(BASE_DIR, 'results', 'propwingtrim_results')
    all_plots(db_name=db_name,
              wingpropinfo=PROWIM_wingpropinfo,
              savedir=savepath)
    stackedplots_wing(db_name=db_name,
                wingpropinfo=PROWIM_wingpropinfo,
                savedir=savepath)
    stackedplots_prop(db_name=db_name,
                    wingpropinfo=PROWIM_wingpropinfo,
                    savedir=savepath)