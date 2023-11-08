# --- Built-ins ---
from pathlib import Path
import os
import logging
import json

# --- Internal ---
from src.base import ParamInfo, WingPropInfo, WingInfo, PropInfo, AirfoilInfo
from src.utils.tools import print_results
from src.postprocessing.plots import all_plots, stackedplots_wing
from src.integration.coupled_groups_optimisation_new import WingSlipstreamPropOptimisationTest
from src.integration.coupled_groups_optimisation import WingSlipstreamPropOptimisation
from src.integration.coupled_groups_analysis import WingSlipstreamPropAnalysis

# --- External ---
import openmdao.api as om
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import niceplots


logging.getLogger('matplotlib.font_manager').disabled = True
BASE_DIR = Path(__file__).parents[0]

prop_radius = 0.1185
# === Read in PROWIM data ===
with open(os.path.join(BASE_DIR, 'data', 'PROWIM.json'), 'r') as file:
    data = json.load(file)

prop_radius = 0.1185
ref_point = data['ref_point']
span = data['span']
twist = data['twist']
chord = data['chord']

alpha_0 = data['alpha_0']
alpha_L0 = data['alpha_L0']
Cl_alpha = data['Cl_alpha']
M = data['M']

wing_twist = 0.
wing_chord = 0.24
wingspan = 0.73*2.*0.952

air_density = 1.2087

spanwise_discretisation_propeller_BEM = len(span)

prop1 = PropInfo(label='Prop1',
                 prop_location=-0.332,
                 nr_blades=4,
                 rot_rate=300.,
                 chord=np.array(chord, order='F')*prop_radius,
                 twist=np.array(twist, order='F'),
                 span=np.array(span, order='F'),
                 airfoils=[AirfoilInfo(label=f'Foil_{index}',
                                       Cl_alpha=Cl_alpha[index],
                                       alpha_L0=alpha_L0[index],
                                       alpha_0=alpha_0[index],
                                       M=M[index])
                           for index in range(spanwise_discretisation_propeller_BEM+1)],
                 ref_point=np.array(ref_point),
                 hub_orientation=np.array([[1.0, 0.0, 0.0],
                                           [0.0, 1.0*np.cos(np.deg2rad(-0.2)),
                                            1.0*np.sin(np.deg2rad(-0.2))],
                                           [0.0, -1.0*np.sin(np.deg2rad(-0.2)), 1.0*np.cos(np.deg2rad(-0.2))]])
                 )

prop2 = PropInfo(label='Prop1',
                 prop_location=0.332,
                 nr_blades=4,
                 rot_rate=300.,
                 chord=np.array(chord, order='F')*prop_radius,
                 twist=np.array(twist, order='F'),
                 span=np.array(span, order='F'),
                 airfoils=[AirfoilInfo(label=f'Foil_{index}',
                                       Cl_alpha=Cl_alpha[index],
                                       alpha_L0=alpha_L0[index],
                                       alpha_0=alpha_0[index],
                                       M=M[index])
                           for index in range(spanwise_discretisation_propeller_BEM+1)],
                 ref_point=ref_point,
                 hub_orientation=np.array([[1.0, 0.0, 0.0],
                                           [0.0, 1.0*np.cos(np.deg2rad(-0.2)),
                                            1.0*np.sin(np.deg2rad(-0.2))],
                                           [0.0, -1.0*np.sin(np.deg2rad(-0.2)), 1.0*np.cos(np.deg2rad(-0.2))]])
                 )

parameters = ParamInfo(vinf=40.,
                       wing_aoa=0.,
                       mach_number=0.2,
                       reynolds_number=640_000,
                       speed_of_sound=333.4,
                       air_density=air_density)


wing = WingInfo(label='PROWIM_wing',
                span=wingspan,
                chord=np.ones(spanwise_discretisation_propeller_BEM,
                              order='F')*wing_chord,
                twist=np.ones(spanwise_discretisation_propeller_BEM,
                              order='F')*wing_twist,
                thickness=np.ones(spanwise_discretisation_propeller_BEM,
                              order='F')*0.01,
                empty_weight=0.,
                # CL0 = 0.27,
                # CD0 = 0.0206
                )


PROWIM_wingpropinfo = WingPropInfo(spanwise_discretisation_propeller=17,
                                    gamma_dphi=10,
                                    spanwise_discretisation_propeller_BEM=spanwise_discretisation_propeller_BEM,
                                    propeller=[prop1, prop2],
                                    wing=wing,
                                    parameters=parameters
                                    )

if __name__ == '__main__':
    # === Load in (experimental) validation data ===
    validationsetup_file = os.path.join(BASE_DIR, 'data', 'PROWIM_validation_conventional.txt')
    validationsetup_data = pd.read_csv(validationsetup_file, delimiter=',', skiprows=22)
    
    # Validation data for J=inf (prop-off)
    n=0
    index1 = n*19
    index2 = (n+1)*19
    aoa = validationsetup_data['AoA'][index1:index2]
    CL_Jinf = validationsetup_data['CL'][index1:index2]
    CD_Jinf = validationsetup_data['CD'][index1:index2]
    
    J_inf = validationsetup_data['J'][index1+1]
    
    # Validation data for J=1
    n=2
    index1 = n*19
    index2 = (n+1)*19
    aoa = validationsetup_data['AoA'][index1:index2]
    CL_J1 = validationsetup_data['CL'][index1:index2]
    CD_J1 = validationsetup_data['CD'][index1:index2]
    J_1 = validationsetup_data['J'][index1+1]
    
    # Validation data for J=0.796
    n=3
    index1 = n*19
    index2 = (n+1)*19
    aoa = validationsetup_data['AoA'][index1:index2]
    CL_J0796 = validationsetup_data['CL'][index1:index2]
    CD_J0796 = validationsetup_data['CD'][index1:index2]
    J_0796 = validationsetup_data['J'][index1+1]
    
    # Validation data for J=0.696
    n=4
    index1 = n*19
    index2 = (n+1)*19
    aoa = validationsetup_data['AoA'][index1:index2]
    CL_J0696 = validationsetup_data['CL'][index1:index2]
    CD_J0696 = validationsetup_data['CD'][index1:index2]
    J_0696 = validationsetup_data['J'][index1+1]
    
    rhoinf = validationsetup_data['rhoInf'][0]
    Vinf = validationsetup_data['Vinf'][0]
    qinf = 0.5*rhoinf*Vinf**2

    angles = np.linspace(-10, 15, 10)
    
    # === Setup PROWIM test case ===
    PROWIM_wingpropinfo.wing.empty_weight = 5 # to make T=D
    PROWIM_wingpropinfo.gamma_tangential_dx = 0.1
    PROWIM_wingpropinfo.gamma_dphi = 10
    
    J = np.array([0.696, 0.796, 0.896])#, 0.696, 0.8960, float('nan')])
    rot_rate = (PROWIM_wingpropinfo.parameters.vinf/(J*2.*prop_radius)) * 2.*np.pi # in rad/s
    
    for iprop, _ in enumerate(PROWIM_wingpropinfo.propeller):
        PROWIM_wingpropinfo.propeller[iprop].rot_rate = rot_rate[0]

    PROWIM_wingpropinfo.__post_init__()
    
    plt.style.use(niceplots.get_style())
    _, ax = plt.subplots(1, figsize=(12, 9))
    
    for index, (irot_rate, num) in enumerate(zip(rot_rate, [7, 8, 9])):#, 8, 9]):
    # for irot_rate, num in zip([rot_rate[0]], [7]):
        T, D, CL, CX, CD = [], [], [], [], []
    
        validation_file = os.path.join(BASE_DIR, 'data', f'exp_data_conv_J0{num}.txt')
        validation_CL_CX_data = pd.read_csv(validation_file, delimiter=',')
        CX_validation = validation_CL_CX_data[f'CX_J=0.{num}']
        CL_validation = validation_CL_CX_data[f'CL_J=0.{num}']
        
        for iprop, _ in enumerate(PROWIM_wingpropinfo.propeller):
            PROWIM_wingpropinfo.propeller[iprop].rot_rate = irot_rate
        
        for angle in angles:
            print(f'Angle of attack: {angle: ^10}')
            if J[index]==0.696:
                PROWIM_wingpropinfo.NO_CORRECTION=False
                PROWIM_wingpropinfo.NO_PROPELLER=False
                # PROWIM_wingpropinfo.wing.CL0 = 0.25#3227
                PROWIM_wingpropinfo.wing.CD0 = 0.025
                
            elif J[index]==0.796:
                PROWIM_wingpropinfo.NO_CORRECTION=False
                PROWIM_wingpropinfo.NO_PROPELLER=False
                # PROWIM_wingpropinfo.wing.CL0 = 0.25#3079
                PROWIM_wingpropinfo.wing.CD0 = 0.025
                
            elif J[index]==0.896:
                PROWIM_wingpropinfo.NO_CORRECTION=False
                PROWIM_wingpropinfo.NO_PROPELLER=False
                # PROWIM_wingpropinfo.wing.CL0 = 0.25#2938
                PROWIM_wingpropinfo.wing.CD0 = 0.025
            PROWIM_wingpropinfo.parameters.wing_aoa = angle

            prob = om.Problem()
            prob.model = WingSlipstreamPropOptimisation(WingPropInfo=PROWIM_wingpropinfo,
                                                            objective={},
                                                            constraints={},
                                                            design_vars={})

            # === Analysis ===
            prob.setup()
            prob.run_model()
        
            T = prob['HELIX_COUPLED.thrust_total']/1.1 # divide by 1.1 to account for thrust overprediction due to region with negative thrust
            S_ref = prob['OPENAEROSTRUCT.AS_point_0.wing_perf.S_ref']#-0.06995*0.24*2 # correction for nacelle
            D = prob['OPENAEROSTRUCT.AS_point_0.total_perf.CD']*qinf*S_ref
            
            CL.append(prob['OPENAEROSTRUCT.AS_point_0.total_perf.CL'])
            CD.append(prob['OPENAEROSTRUCT.AS_point_0.total_perf.CD'])
            cx = (D-T)/(qinf*S_ref)
            CX.append(cx)
            
            # Cl = prob['OPENAEROSTRUCT.AS_point_0.wing_perf.Cl']
            # spanwwise = np.linspace(-0.5, 0.5, len(Cl))
            # plt.plot(spanwwise, Cl)
            # plt.show()
            
            # quit()
        
        # ax[0].plot(angles, T, label=r'$T$', color='b')
        # ax[0].plot(angles, D, label=r'$D$', color='orange')
        ax.scatter(CX_validation, CL_validation, label=f'Exp., J=0.{num}')
        ax.plot(CX, CL, label=f'Num., J=0.{num}', linestyle='dashed')
        ax.set_ylabel(r'$C_L (-)$')
        ax.set_xlabel(r'$C_X (-)$')
        ax.set_ylim((-0.4, 1.6))
        ax.set_xlim((-0.2, 0.1))
        ax.legend()
        
        # ax[1].plot(angles, CL, label=f'Num, J=0.{num}')
        # ax[1].scatter(aoa, CL_J0796, label=f'Exp, J=0.7')
        # ax[1].set_ylabel(r'$C_L (-)$')
        # ax[1].set_xlabel(r'$AoA (deg)$')
        # # ax[0].set_ylim((-0.4, 1.6))
        # # ax[0].set_xlim((-0.2, 0.))
        # ax[1].legend()
    
    niceplots.adjust_spines(ax, outward=True)
    # niceplots.adjust_spines(ax[1], outward=True)
    
    plt.savefig(os.path.join(BASE_DIR, 'figures', 'TUBE_PROWIM_VALIDATION.png'))
    
    plt.clf()
    plt.close()
    
    print(angles, CL)
