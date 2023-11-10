# --- Built-ins ---
from pathlib import Path
import os
import logging
import copy

# --- Internal ---
from src.utils.tools import print_results
from src.postprocessing.plots import all_plots, stackedplots_prop
from src.integration.coupled_groups_optimisation import PropOptimisation
from examples.example_classes.PROWIM_classes import PROWIM_wingpropinfo, PROWIM_prop_1, PROWIM_parameters

# --- External ---
import openmdao.api as om
import numpy as np
import matplotlib.pyplot as plt
import niceplots


logging.getLogger('matplotlib.font_manager').disabled = True
BASE_DIR = Path(__file__).parents[0]

def DEP_comparison(thrust: float, radius: float, nr_props: int):
    PROWIM_wingpropinfo.propeller = [PROWIM_prop_1]
    PROWIM_wingpropinfo.nr_props = len(PROWIM_wingpropinfo.propeller)
    PROWIM_wingpropinfo.propeller[0].rot_rate = 1060
    PROWIM_wingpropinfo.propeller[0].chord *= 1./nr_props
    PROWIM_wingpropinfo.parameters.air_density = 1.2087
    PROWIM_wingpropinfo.propeller[0].span *= radius/0.1185
    
    objective = {
                'HELIX_COUPLED.power_total':
                    {'scaler': 1}
                }

    design_vars = {
                    'HELIX_0.om_helix.geodef_parametric_0_twist':
                        {'lb': 0,
                        'ub': 90,
                        'scaler': 1./10},
                    'HELIX_0.om_helix.geodef_parametric_0_rot_rate':
                        {'lb': 0,
                        'ub': 3000,
                        'scaler': 1./1060}
                    }

    constraints = {
                    'HELIX_COUPLED.thrust_total':
                        {'equals': thrust/nr_props}
                    }
    
    prob = om.Problem()
    prob.model = PropOptimisation(  WingPropInfo=PROWIM_wingpropinfo,
                                    objective=objective,
                                    constraints=constraints,
                                    design_vars=design_vars)

    # === Optimisation ===
    prob.driver = om.pyOptSparseDriver()
    prob.driver.options['optimizer'] = 'SNOPT'
    prob.driver.options['debug_print'] = ['desvars', 'nl_cons', 'objs']
    prob.driver.opt_settings = {
        "Major feasibility tolerance": 1.0e-5,
        "Major optimality tolerance": 1.0e-5,
        "Minor feasibility tolerance": 1.0-5,
        "Verify level": -1,
        "Function precision": 1.0e-6,
        # "Major iterations limit": 1,
        "Nonderivative linesearch": None,
        "Print file": os.path.join(BASE_DIR, 'results', 'optimisation_proponly_print.out'),
        "Summary file": os.path.join(BASE_DIR, 'results', 'optimisation_proponly_summary.out')
    }
    
    print('==========================================================')
    print('====================== Optimisation ======================')
    print('==========================================================')
    prob.setup(mode='rev')    
    prob.run_driver()
    
    print_results(design_vars=design_vars, constraints=constraints, objective=objective,
                  prob=prob, kind="Results")
    
    T = prob["HELIX_0.om_helix.rotorcomp_0_thrust"][2, 0]
    P = prob["HELIX_0.om_helix.rotorcomp_0_power"][0]
    eta = T*PROWIM_wingpropinfo.parameters.vinf/P
    print(np.round(T,2), np.round(P,2), np.round(eta,2))
    
    PROWIM_wingpropinfo.propeller[0].chord *= nr_props
    PROWIM_wingpropinfo.propeller[0].span *= 0.1185/radius
    
    return T, P, eta, prob['HELIX_0.om_helix.geodef_parametric_0_rot_rate']

if __name__=='__main__':
    thrust = 100
    nr_radii = 3
    radius = 0.1185*4
    T_lst, P_lst, eta_lst, rot_rate_lst = \
        [], [], [], []
    
    for iradius in range(1, nr_radii+1):
        radius_tmp = radius/iradius
        try:
            T, P, eta, rot_rate = DEP_comparison(thrust=thrust, radius=radius_tmp, nr_props=iradius)
            T_lst.append(T)
            P_lst.append(P)
            eta_lst.append(eta)
            rot_rate_lst.append(rot_rate)
        except Exception as e:
            raise AssertionError(e)
        
    nr_props = np.arange(1, nr_radii+1)
    
    # === Plot results ===
    plt.style.use(niceplots.get_style())
    plt.rc('font', size=10)
    _, ax = plt.subplots(nr_radii+1, figsize=(7, 7), sharex=True)
    
    ax[0].plot(nr_props, T_lst, label=r'Thrust')    
    ax[0].legend(fontsize='12')
    ax[0].set_ylabel(r'Thrust, N')
    niceplots.adjust_spines(ax[0], outward=True)
    
    ax[1].plot(nr_props, P_lst, label=r'Power')    
    ax[1].legend(fontsize='12')
    ax[1].set_ylabel('Power, W')
    niceplots.adjust_spines(ax[1], outward=True)
    
    ax[2].plot(nr_props, eta_lst, label=r'Eta')    
    ax[2].legend(fontsize='12')
    ax[2].set_ylabel('Eta, -')
    niceplots.adjust_spines(ax[2], outward=True)
    
    ax[3].plot(nr_props, rot_rate_lst, label=r'Rot rate')    
    ax[3].legend(fontsize='12')
    ax[3].set_ylabel('Rot rate, rad/s')
    ax[3].set_xlabel('Number of propellers')
    niceplots.adjust_spines(ax[3], outward=True)

    plt.savefig(os.path.join(BASE_DIR, 'figures', 'DEP_comparison_BEM.png'))