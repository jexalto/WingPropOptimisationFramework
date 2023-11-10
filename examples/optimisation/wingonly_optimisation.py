# --- Built-ins ---
from pathlib import Path
import os
import logging
import copy

# --- Internal ---
from src.utils.tools import print_results
from src.postprocessing.plots import all_plots, stackedplots_wing
from src.integration.coupled_groups_optimisation import WingOptimisation
from examples.example_classes.PROWIM_classes import PROWIM_wingpropinfo

# --- External ---
import openmdao.api as om
import numpy as np


logging.getLogger('matplotlib.font_manager').disabled = True
BASE_DIR = Path(__file__).parents[0]

if __name__ == '__main__':
    
    # db_name = os.path.join(BASE_DIR, 'results', 'data_wing.db')
    # savepath = os.path.join(BASE_DIR, 'results', 'wing_results')
    # stackedplots_wing(db_name=db_name,
    #             wingpropinfo=PROWIM_wingpropinfo,
    #             savedir=savepath,
    #             noprop=True)
    # quit()

    Preq = 0
    mbat = 4
    PROWIM_wingpropinfo.spanwise_discretisation_propeller = 11
    PROWIM_wingpropinfo.__post_init__()
    PROWIM_wingpropinfo.wing.empty_weight = mbat + Preq/PROWIM_wingpropinfo.propeller[0].esp # weight model
    PROWIM_wingpropinfo.wing.CL0 = 0.
    # PROWIM_wingpropinfo.wing.span *= 1
    # PROWIM_wingpropinfo.linear_mesh = True
    # PROWIM_wingpropinfo.__post_init__()
    
    # PROWIM_wingpropinfo.wing.twist = np.array([-0.47012745,  0.57791105,  2.50152062,  4.38360291,  5.9963285 ,
    #                                             7.08826306,  7.81453015,  6.49637919,  7.80467608,  7.1034328 ,
    #                                             5.98106769,  4.35521603,  2.47558433,  0.55751195, -0.47555508])
    # PROWIM_wingpropinfo.wing.chord =  np.array([0.08329586, 0.08340191, 0.0832217 , 0.08340579, 0.08323511,
    #                                             0.08352932, 0.08280204, 0.09805698, 0.08257516, 0.08350386,
    #                                             0.08337559, 0.08324213, 0.08337507, 0.08330022, 0.08326784])
    
    objective = {
                'OPENAEROSTRUCT.AS_point_0.total_perf.D':
                    {'scaler': 1/2.66093432}
                }

    design_vars = {
                    'OPENAEROSTRUCT.wing.twist_cp':
                        {'lb': -10,
                        'ub': 8,
                        'scaler': 1},
                    'OPENAEROSTRUCT.wing.geometry.chord_cp':
                        {'lb': 0.01,
                        'ub': 30,
                        'scaler': 100},
                    'OPENAEROSTRUCT.wing.thickness_cp':
                        {'lb': 3e-3,
                        'ub': 2e-1,
                        'scaler': 1/3e-3}
                    }

    constraints = {
                    'OPENAEROSTRUCT.AS_point_0.total_perf.CL':
                        {'upper': 0.8},
                    'OPENAEROSTRUCT.AS_point_0.L_equals_W':
                        {'equals': 0.},
                    'OPENAEROSTRUCT.AS_point_0.wing_perf.failure':
                        {'upper': 0.},
                    'OPENAEROSTRUCT.AS_point_0.wing_perf.thickness_intersects':
                        {'upper': 0.},
                    'OPENAEROSTRUCT.wing.structural_mass':
                        {'lower': 0.},
                    'OPENAEROSTRUCT.AS_point_0.total_perf.L':
                        {'lower': 0.},
                    }
    
    prob = om.Problem()
    prob.model = WingOptimisation(  WingPropInfo=PROWIM_wingpropinfo,
                                    objective=objective,
                                    constraints=constraints,
                                    design_vars=design_vars)
    
    # === Analysis ===
    prob.setup()
    prob.run_model()
    
    prob.check_totals(compact_print=True,
                      show_only_incorrect=True)
    
    quit()
    om.n2(prob, 'wingonly_opt.html')
    
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
    # "Major iterations limit": 2,
    "Nonderivative linesearch": None,
    "Print file": os.path.join(BASE_DIR, 'results', 'optimisation_print_wing.out'),
    "Summary file": os.path.join(BASE_DIR, 'results', 'optimisation_summary_wing.out')
    }
    
        # Initialise recorder
    db_name = os.path.join(BASE_DIR, 'results', 'data_wing.db')
    
    recorder = om.SqliteRecorder(db_name)
    prob.driver.add_recorder(recorder)
    prob.driver.add_recorder(recorder)
    prob.driver.recording_options['includes'] = [
                                                    # "OPENAEROSTRUCT.wing.mesh",
                                                    "OPENAEROSTRUCT.wing.geometry.twist",
                                                    "OPENAEROSTRUCT.wing.geometry.chord",
                                                    "OPENAEROSTRUCT.AS_point_0.wing_perf.Cl",
                                                    "OPENAEROSTRUCT.AS_point_0.total_perf.L",
                                                    "OPENAEROSTRUCT.AS_point_0.total_perf.D",
                                                ]
    
    print('==========================================================')
    print('====================== Optimisation ======================')
    print('==========================================================')
    prob.setup()
    prob.run_driver()
    
    print_results(design_vars=design_vars, constraints=constraints, objective=objective,
                  prob=prob, kind="Optimisation")
    
    savepath = os.path.join(BASE_DIR, 'results', 'wing_results')
    all_plots(db_name=db_name,
              wingpropinfo=PROWIM_wingpropinfo,
              savedir=savepath)
    
    stackedplots_wing(db_name=db_name,
                wingpropinfo=PROWIM_wingpropinfo,
                savedir=savepath,
                noprop=True)