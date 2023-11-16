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
    #             savedir=savepath)
    # quit()

    PROWIM_wingpropinfo.wing.empty_weight = 5 # to make T=D
    PROWIM_wingpropinfo.wing.CL0 = 0. # to make T=D
    PROWIM_wingpropinfo.tubemodelON = False
    
    PROWIM_wingpropinfo.__post_init__()
    
    objective = {
                'OPENAEROSTRUCT.AS_point_0.total_perf.D':
                    {'scaler': 1/9.81879759}
                }

    design_vars = {
                    'OPENAEROSTRUCT.wing.twist_cp':
                        {'lb': -10,
                        'ub': 8,
                        'scaler': 1},
                    'OPENAEROSTRUCT.wing.geometry.chord_cp':
                        {'lb': 0.01,
                        'ub': 30,
                        'scaler': 10},
                    'OPENAEROSTRUCT.wing.thickness_cp':
                        {'lb': 3e-3,
                        'ub': 2e-1,
                        'scaler': 1e2}
                    }

    constraints = {
                    'OPENAEROSTRUCT.AS_point_0.total_perf.CL':
                        {'upper': 1.},
                    'OPENAEROSTRUCT.AS_point_0.L_equals_W':
                        {'equals': 0.},
                    'OPENAEROSTRUCT.AS_point_0.wing_perf.failure':
                        {'upper': 0.},
                    'OPENAEROSTRUCT.AS_point_0.wing_perf.thickness_intersects':
                        {'upper': 0.},
                    }
    
    prob = om.Problem()
    prob.model = WingOptimisation(  WingPropInfo=PROWIM_wingpropinfo,
                                    objective=objective,
                                    constraints=constraints,
                                    design_vars=design_vars)
    
    # === Analysis ===
    prob.setup()
    prob.run_model()
    om.n2(prob, 'wingonly_opt.html')
    
    print_results(design_vars=design_vars, constraints=constraints, objective=objective,
                  prob=prob, kind="Initial Analysis")

    # === Optimisation ===
    prob.driver = om.pyOptSparseDriver()
    prob.driver.options['optimizer'] = 'SNOPT'
    prob.driver.options['debug_print'] = ['desvars', 'nl_cons']
    prob.driver.opt_settings = {
    "Major feasibility tolerance": 1.0e-8,
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
                                                    "OPENAEROSTRUCT.wing.mesh",
                                                    "OPENAEROSTRUCT.wing.geometry.twist",
                                                    "OPENAEROSTRUCT.wing.geometry.chord",
                                                    "OPENAEROSTRUCT.AS_point_0.wing_perf.Cl"
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