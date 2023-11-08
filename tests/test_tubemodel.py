# --- Built-ins ---
from pathlib import Path
import os
import logging
import copy

# --- Internal ---
from src.utils.tools import print_results
from src.postprocessing.plots import all_plots, stackedplots_wing
from src.integration.coupled_groups_optimisation_new import WingSlipstreamPropOptimisationTest
from examples.example_classes.PROWIM_classes import PROWIM_wingpropinfo

# --- External ---
import openmdao.api as om
import numpy as np


logging.getLogger('matplotlib.font_manager').disabled = True
BASE_DIR = Path(__file__).parents[0]

if __name__ == '__main__':
    # # === Plotting ===
    # db_name = os.path.join(BASE_DIR, 'results', 'data_wingprop.db')
    # savepath = os.path.join(BASE_DIR, 'results', 'propwing_results')
    # stackedplots_wing(db_name=db_name,
    #             wingpropinfo=PROWIM_wingpropinfo,
    #             savedir=savepath)
    # quit()

    PROWIM_wingpropinfo.spanwise_discretisation_propeller = 13 # to make T=D
    PROWIM_wingpropinfo.wing.empty_weight = 5 # to make T=D
    PROWIM_wingpropinfo.wing.CL0 = 0. # to make T=D
    PROWIM_wingpropinfo.gamma_tangential_dx = 0.1
    PROWIM_wingpropinfo.wing.youngsmodulus=1e16
    PROWIM_wingpropinfo.wing.G=1e16
    PROWIM_wingpropinfo.NO_CORRECTION = False
    PROWIM_wingpropinfo.force = np.zeros(76)
    
    for index in range(len(PROWIM_wingpropinfo.propeller)):
        PROWIM_wingpropinfo.propeller[index].rot_rate = 1060
        PROWIM_wingpropinfo.propeller[index].twist = np.array([ 67.20825134, 61.01531338, 55.52115905, 50.70011031, 46.54195678, 41.28970741,
                                                                37.49211551, 34.56964193, 32.0884532,  29.94856091, 28.10354481, 26.48669457,
                                                                25.07074381, 23.80806013, 22.69461815, 21.71183634, 20.86579719, 20.16675169,
                                                                19.61528706, 19.43809687])

    PROWIM_wingpropinfo.__post_init__()
    
    objective = {
                'HELIX_COUPLED.power_total':
                    {'scaler': 1/433.04277037}
                }

    design_vars = {
                    'DESIGNVARIABLES.rotor_0_rot_rate':
                        {'lb': 0,
                        'ub': 3000,
                        'scaler': 1./1060},
                    'DESIGNVARIABLES.rotor_1_rot_rate':
                        {'lb': 0,
                        'ub': 3000,
                        'scaler': 1./1060},
                    'DESIGNVARIABLES.rotor_0_twist':
                        {'lb': 0,
                        'ub': 90,
                        'scaler': 1./10},
                    'DESIGNVARIABLES.rotor_1_twist':
                        {'lb': 0,
                        'ub': 90,
                        'scaler': 1./10},
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
                        'ub': 5e-1,
                        'scaler': 1e2},
                    }

    constraints = {
                    'OPENAEROSTRUCT.AS_point_0.wing_perf.failure':
                        {'upper': 0.},
                    'OPENAEROSTRUCT.AS_point_0.total_perf.CL':
                        {'upper': 1.},
                    'OPENAEROSTRUCT.AS_point_0.wing_perf.thickness_intersects':
                        {'upper': 0.},
                    'OPENAEROSTRUCT.AS_point_0.L_equals_W':
                        {'equals': 0.},
                    'CONSTRAINTS.thrust_equals_drag':
                        {'equals': 0.}
                    }
    
    
    prob = om.Problem()
    prob.model = WingSlipstreamPropOptimisationTest(WingPropInfo=PROWIM_wingpropinfo,
                                                    objective=objective,
                                                    constraints=constraints,
                                                    design_vars=design_vars)
    
    # === Analysis ===
    # prob.setup()
    # prob.run_model()

    # lift_coefficient_withoutprop = prob['OPENAEROSTRUCT.AS_point_0.wing_perf.Cl']

    def parabola(x): return 5-x**2

    PROWIM_wingpropinfo.gamma_dphi = 5
    PROWIM_wingpropinfo.force = parabola(x=np.linspace(-1, 1, 76))
    PROWIM_wingpropinfo.__post_init__()

    prob = om.Problem()
    prob.model = WingSlipstreamPropOptimisationTest(WingPropInfo=PROWIM_wingpropinfo,
                                                    objective=objective,
                                                    constraints=constraints,
                                                    design_vars=design_vars)

    # === Analysis ===
    prob.setup()
    prob.run_model()
    
    if False:
        partials= prob.check_partials(compact_print=True,
                                        show_only_incorrect=True,
                                        includes=[
                                                # '*COUPLED_OAS_TUBE.TUBEMODEL.TUBEMODEL_velocity_output*',
                                                # '*COUPLED_OAS_TUBE.TUBEMODEL.TUBEMODEL_coupled.TUBEMODEL_forceinterpolation_0*',
                                                #   '*COUPLED_OAS_TUBE.TUBEMODEL.TUBEMODEL_coupled.TUBEMODEL_KuttaJoukowski_0*',
                                                #   '*COUPLED_OAS_TUBE.TUBEMODEL.TUBEMODEL_coupled.TUBEMODEL_biotsavart_0*'
                                                '*OPENAEROSTRUCT.wing*',
                                                '*CONSTRAINTS*',
                                                '*OPENAEROSTRUCT.AS_point_0.total_perf*',
                                                '*COUPLED_OAS_TUBE.OPENAEROSTRUCT.AS_point_0.wing_perf.struct_funcs.failure*',
                                                
                                                ],
                                        step=1e-8,
                                        form='central',
                                        rel_err_tol=1e-3)
        # prob.check_totals(compact_print=True,
        #                   show_only_incorrect=True,
        #                   step=1e-8,
        #                 form='central',)
        quit()
        
    lift_coefficient_withprop = prob['OPENAEROSTRUCT.AS_point_0.wing_perf.Cl']
    
    wingspan = PROWIM_wingpropinfo.vlm_mesh_control_points
    
    import matplotlib.pyplot as plt
    import niceplots
    
    plt.style.use(niceplots.get_style())
    veldistr_x = prob['TUBEMODEL.TUBEMODEL_velocity_output.velocity_vector'][:, 0]
    veldistr_y = prob['TUBEMODEL.TUBEMODEL_velocity_output.velocity_vector'][:, 1]
    veldistr_z = prob['TUBEMODEL.TUBEMODEL_velocity_output.velocity_vector'][:, 2]
    
    _, ax = plt.subplots(1, figsize=(10, 5), sharex=True)
    # ax.plot(wingspan, lift_coefficient_withoutprop, label='No prop', linestyle='dashed', color='black', linewidth=1)
    ax.plot(wingspan, lift_coefficient_withprop, label='With prop', linewidth=2)
    ax.set_ylabel(r'$C_l$')
    ax.set_xlabel(r'$Span$')
    ax.legend()
    
    plt.savefig('propnoprop.png')
    plt.clf()
    
    # veldistr_x = prob['OPENAEROSTRUCT.AS_point_0.coupled.aero_states.freestream_velocities'][:, 0]
    # veldistr_y = prob['OPENAEROSTRUCT.AS_point_0.coupled.aero_states.freestream_velocities'][:, 1]
    # veldistr_z = prob['OPENAEROSTRUCT.AS_point_0.coupled.aero_states.freestream_velocities'][:, 2]
    
    _, ax = plt.subplots(4, figsize=(12, 8), sharex=True)
    for index, (veldistr, ylabel) in enumerate(zip([veldistr_x, veldistr_y, veldistr_z, lift_coefficient_withprop],
                                               [r'$\Delta V_x$',
                                                r'$\Delta V_y$',
                                                r'$\Delta V_z$',
                                                r'$C_l$'])):
        ax[index].plot(wingspan, veldistr)       
        ax[index].set_ylabel(ylabel)
        niceplots.adjust_spines(ax[index], outward=True)
        # ax[index].set_ylim(min(veldistr)*1.1, max(veldistr)/0.9)

    plt.savefig('vel_distr.png')
    
    quit()
    
    # === Optimization ===

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

    # === Optimisation ===
    prob.driver = om.pyOptSparseDriver()
    prob.driver.options['optimizer'] = 'SNOPT'
    prob.driver.options['debug_print'] = ['desvars', 'nl_cons']
    prob.driver.opt_settings = {
        "Major feasibility tolerance": 1.0e-9,
        "Major optimality tolerance": 1.0e-9,
        "Minor feasibility tolerance": 1.0e-8,
        "Verify level": -1,
        "Function precision": 1.0e-6,
        "Major iterations limit": 0,
        "Nonderivative linesearch": None,
        "Print file": os.path.join(BASE_DIR, 'results', 'optimisation_print_wingprop_tube.out'),
        "Summary file": os.path.join(BASE_DIR, 'results', 'optimisation_summary_wingprop_tube.out')
    }
    
    # prob.check_totals(  compact_print=True, show_only_incorrect=True,
    #                     form='central', step=1e-8, 
    #                     rel_err_tol=1e-3)
    
        # Initialise recorder
    db_name = os.path.join(BASE_DIR, 'results', 'data_wingprop.db')
    
    includes = ["OPENAEROSTRUCT.wing.geometry.twist",
                "OPENAEROSTRUCT.wing.geometry.chord",
                "OPENAEROSTRUCT.AS_point_0.wing_perf.Cl",
                "OPENAEROSTRUCT.AS_point_0.wing_perf.CDi",
                'OPENAEROSTRUCT.AS_point_0.total_perf.L',
                'OPENAEROSTRUCT.AS_point_0.total_perf.D',
                'RETHORST.velocity_distribution',
                'propeller_velocity']
    
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
    savepath = os.path.join(BASE_DIR, 'results', 'propwing_results')
    all_plots(db_name=db_name,
              wingpropinfo=PROWIM_wingpropinfo,
              savedir=savepath)
    stackedplots_wing(db_name=db_name,
                wingpropinfo=PROWIM_wingpropinfo,
                savedir=savepath)
