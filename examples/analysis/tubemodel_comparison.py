# --- Built-ins ---
from pathlib import Path
import os
import logging
import copy

# --- Internal ---
from src.utils.tools import print_results
from src.postprocessing.plots import all_plots, stackedplots_wing, stackedplots_prop
from src.integration.coupled_groups_optimisation import WingSlipstreamPropOptimisation, WingRethorstPropOptimisation
from examples.example_classes.PROWIM_classes import PROWIM_wingpropinfo

# --- External ---
import openmdao.api as om
import numpy as np
import matplotlib.pyplot as plt


logging.getLogger('matplotlib.font_manager').disabled = True
BASE_DIR = Path(__file__).parents[0]

if __name__ == '__main__':
    PROWIM_wingpropinfo.spanwise_discretisation_propeller = 19
    PROWIM_wingpropinfo.gamma_dphi = 10
    PROWIM_wingpropinfo.gamma_tangential_dx = 0.1
    # PROWIM_wingpropinfo.linear_mesh=True
    # PROWIM_wingpropinfo.__post_init__()

    PROWIM_wingpropinfo.wing.twist = np.array([-0.46449103,  0.59262155,  2.52644791,  4.41827328,  6.03133641,
                                               7.12311048,  7.8495755,  6.53179668,  7.83959416,  7.13808015,
                                               6.01575541,  4.38942103,  2.50011456,  0.57197008, -0.47001715])
    PROWIM_wingpropinfo.wing.chord = np.array([0.08333331, 0.08333353, 0.08333255, 0.08333556, 0.08332314,
                                               0.08340374, 0.08279963, 0.09822044, 0.08279963, 0.08340374,
                                               0.08332314, 0.08333556, 0.08333255, 0.08333353, 0.08333331])

    for iprop, _ in enumerate(PROWIM_wingpropinfo.propeller):
        PROWIM_wingpropinfo.propeller[iprop].rot_rate = 800
        PROWIM_wingpropinfo.propeller[iprop].chord *= 1.
        PROWIM_wingpropinfo.propeller[iprop].span *= 1
        PROWIM_wingpropinfo.propeller[iprop].twist = np.array([85.42133744, 83.78989181, 81.90467817, 80.11006664, 78.65620662,
                                                               77.34008956, 77.22522702, 74.78657488, 74.66329572, 72.81191379,
                                                               70.5673611, 70.82226808, 66.86461581, 67.74485409, 64.26294555,
                                                               63.82034587, 62.18677825, 59.68273323, 59.69917405, 55.07547007])

    objective = {
        'HELIX_COUPLED.power_total':
        {'scaler': 1/112}
    }

    design_vars = {
        'DESIGNVARIABLES.rotor_0_rot_rate':
        {'lb': 0,
         'ub': 3000,
         'scaler': 1./244.68179184},
        'DESIGNVARIABLES.rotor_1_rot_rate':
        {'lb': 0,
         'ub': 3000,
         'scaler': 1./244.68179184},
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
         'scaler': 1/0.083},
        'OPENAEROSTRUCT.wing.thickness_cp':
        {'lb': 3e-3,
         'ub': 5e-1,
         'scaler': 1/3e-3},
    }

    constraints = {
        'OPENAEROSTRUCT.AS_point_0.wing_perf.failure':
        {'upper': 0.},
        'OPENAEROSTRUCT.AS_point_0.total_perf.CL':
        {'upper': 0.8},
        'OPENAEROSTRUCT.AS_point_0.wing_perf.thickness_intersects':
        {'upper': 0.},
        'OPENAEROSTRUCT.AS_point_0.L_equals_W':
        {'equals': 0.},
        'HELIX_COUPLED.thrust_total':
        {'lower': 0.},
        'CONSTRAINTS.thrust_equals_drag':
        {'equals': 0.},
        'OPENAEROSTRUCT.wing.structural_mass':
        {'lower': 0.},
        'OPENAEROSTRUCT.AS_point_0.total_perf.D':
        {'lower': 0.},
        'OPENAEROSTRUCT.AS_point_0.total_perf.L':
        {'lower': 0.},
        'OPENAEROSTRUCT.W0':
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
    om.n2(prob)

    print('Original: ', prob["OPENAEROSTRUCT.AS_point_0.total_perf.D"])

    # prob = om.Problem()
    # prob.model = WingRethorstPropOptimisation(WingPropInfo=PROWIM_wingpropinfo,
    #                                             objective=objective,
    #                                             constraints=constraints,
    #                                             design_vars=design_vars)

    # # === Analysis ===
    # prob.setup()
    # prob.run_model()
    # om.n2(prob)

    # CL_tubemodel = prob["OPENAEROSTRUCT.AS_point_0.wing_perf.Cl"]

    # print('Tubemodel: ', prob["OPENAEROSTRUCT.AS_point_0.total_perf.D"])

    # wingspan = PROWIM_wingpropinfo.vlm_mesh_control_points

    # import matplotlib.pyplot as plt
    # import niceplots

    # plt.style.use(niceplots.get_style())
    # veldistr_x = prob['COUPLED_OAS_TUBE.TUBEMODEL.TUBEMODEL_velocity_output.velocity_vector'][:, 0]
    # veldistr_y = prob['COUPLED_OAS_TUBE.TUBEMODEL.TUBEMODEL_velocity_output.velocity_vector'][:, 1]
    # veldistr_z = prob['COUPLED_OAS_TUBE.TUBEMODEL.TUBEMODEL_velocity_output.velocity_vector'][:, 2]

    # # veldistr_x = prob['OPENAEROSTRUCT.AS_point_0.coupled.aero_states.freestream_velocities'][:, 0]
    # # veldistr_y = prob['OPENAEROSTRUCT.AS_point_0.coupled.aero_states.freestream_velocities'][:, 1]
    # # veldistr_z = prob['OPENAEROSTRUCT.AS_point_0.coupled.aero_states.freestream_velocities'][:, 2]

    # _, ax = plt.subplots(4, figsize=(12, 8), sharex=True)
    # for index, (veldistr, ylabel) in enumerate(zip([veldistr_x, veldistr_y, veldistr_z, CL_tubemodel],
    #                                            [r'$\Delta V_x$',
    #                                             r'$\Delta V_y$',
    #                                             r'$\Delta V_z$',
    #                                             r'$C_L$'])):
    #     ax[index].plot(wingspan, veldistr)
    #     ax[index].set_ylabel(ylabel)
    #     niceplots.adjust_spines(ax[index], outward=True)

    # plt.savefig('vel_distr.png')

    # Check derivatives
    if True:
        prob.check_totals(compact_print=True, show_only_incorrect=True,
                          form='central', step=1e-8,
                          rel_err_tol=1e-3)
        # partials = prob.check_partials(compact_print=True, show_only_incorrect=True,
        #                                 includes=['*OPENAEROSTRUCT.AS_point_0.total_perf*',
        #                                           '*OPENAEROSTRUCT.AS_point_0.wing_perf*'],
        #                                 form='central', step=1e-6)

        quit()

    quit()
    print_results(design_vars=design_vars, constraints=constraints, objective=objective,
                  prob=prob, kind="Initial Analysis")

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
        "Print file": os.path.join(BASE_DIR, 'results', 'optimisation_print_wingprop.out'),
        "Summary file": os.path.join(BASE_DIR, 'results', 'optimisation_summary_wingprop.out')
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

    prob.cleanup()  # close all recorders

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
    stackedplots_prop(db_name=db_name,
                      wingpropinfo=PROWIM_wingpropinfo,
                      savedir=savepath)
