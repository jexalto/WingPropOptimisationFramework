# --- Built-ins ---
from pathlib import Path
import os
import logging
import copy

# --- Internal ---
from src.utils.tools import print_results
from src.postprocessing.plots import all_plots
from src.integration.coupled_groups_optimisation import PropOptimisation
from examples.example_classes.PROWIM_classes import PROWIM_wingpropinfo, PROWIM_prop_1, PROWIM_parameters

# --- External ---
import openmdao.api as om
import numpy as np


logging.getLogger('matplotlib.font_manager').disabled = True
BASE_DIR = Path(__file__).parents[0]

if __name__=='__main__':
    PROWIM_wingpropinfo.propeller = [PROWIM_prop_1]
    PROWIM_wingpropinfo.nr_props = len(PROWIM_wingpropinfo.propeller)
    PROWIM_wingpropinfo.propeller[0].rot_rate = 249 * 2.0 * np.pi
    PROWIM_wingpropinfo.parameters.vinf = 50
    PROWIM_wingpropinfo.parameters.air_density = 1.2087
    
    objective = {
                'HELIX_COUPLED.power_total':
                    {'scaler': 1/509.77978858}
                }

    design_vars = {
                    'HELIX_0.om_helix.geodef_parametric_0_twist':
                        {'lb': 0,
                        'ub': 90,
                        'scaler': 1./10},
                    'HELIX_0.om_helix.geodef_parametric_0_rot_rate':
                        {'lb': 0,
                        'ub': 3000,
                        'scaler': 1./1060.4532164},
                    'DESIGNVARIABLES.rotor_0_chord':
                        {'lb': -np.inf,
                        'ub': np.inf,
                        'scaler': 1./0.012032137566147693}
                    }

    constraints = {
                    'HELIX_COUPLED.thrust_total':
                        {'equals': 8.47001072}
                    }
    
    prob = om.Problem()
    prob.model = PropOptimisation(  WingPropInfo=PROWIM_wingpropinfo,
                                    objective=objective,
                                    constraints=constraints,
                                    design_vars=design_vars)
    
    prob.setup()
    prob.run_model()
    
    print_results(design_vars=design_vars, constraints=constraints, objective=objective,
                  prob=prob, kind="Initial Analysis")
    
    print('Variables:')
    print(prob["HELIX_0.om_helix.geodef_parametric_0_rot_rate"])
    print(prob["HELIX_0.om_helix.geodef_parametric_0_chord"])
    print(prob["HELIX_0.om_helix.geodef_parametric_0_twist"])
    print(prob["HELIX_0.om_helix.geodef_parametric_0_span"])
    
    quit()
    # === Optimisation ===
    prob.driver = om.pyOptSparseDriver()
    prob.driver.options['optimizer'] = 'SNOPT'
    prob.driver.opt_settings = {
    "Major feasibility tolerance": 1.0e-5,
    "Major optimality tolerance": 1.0e-7,
    "Minor feasibility tolerance": 1.0-5,
    "Verify level": -1,
    "Function precision": 1.0e-6,
    # "Major iterations limit": 50,
    "Nonderivative linesearch": None,
    "Print file": os.path.join(BASE_DIR, 'results', 'optimisation_proponly_print.out'),
    "Summary file": os.path.join(BASE_DIR, 'results', 'optimisation_proponly_summary.out')
    }
        
        # Initialise recorder
    db_name = os.path.join(BASE_DIR, 'results', 'data_propeller.db')
    
    recorder = om.SqliteRecorder(db_name)
    prob.driver.add_recorder(recorder)
    prob.driver.add_recorder(recorder)
    prob.driver.recording_options['includes'] = [
                                                    "HELIX_0.om_helix.rotorcomp_0_radii",
                                                    "HELIX_0.om_helix.rotorcomp_0_velocity_distribution",
                                                    "blade_chord_spline_0.y"
                                                 ]
    
    print('==========================================================')
    print('====================== Optimisation ======================')
    print('==========================================================')
    prob.setup(mode='rev')
    prob.run_driver()
    
    print_results(design_vars=design_vars, constraints=constraints, objective=objective,
                  prob=prob, kind="Initial Analysis")
    
    savepath = os.path.join(BASE_DIR, 'results', 'prop_results')
    all_plots(db_name=db_name,
              wingpropinfo=PROWIM_wingpropinfo,
              savedir=savepath)