# --- Built-ins ---
import os
from pathlib import Path
import json

# --- Internal ---
from src.base import ParamInfo, WingPropInfo, WingInfo, PropInfo, AirfoilInfo
from src.integration.coupled_groups_analysis import WingSlipstreamPropAnalysis

# --- External ---
import numpy as np
import pandas as pd
import openmdao.api as om
import matplotlib.pyplot as plt
import niceplots


BASE_DIR = Path(__file__).parents[0]


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
CD0 = np.array(data['cd0'])
M = data['M']

wing_twist = 0.
wing_chord = 0.24
wingspan = 0.73*2.*0.952

spanwise_discretisation_propeller_BEM = len(span)


prop1 = PropInfo(label='Prop1',
                 prop_location=-0.332,
                 nr_blades=4,
                 rot_rate=300.,
                 chord=np.array(chord, order='F'),
                 twist=np.array(twist, order='F'),
                 span=np.array(span, order='F'),
                 airfoils=[AirfoilInfo(label=f'Foil_{index}',
                                       Cl_alpha=Cl_alpha[index],
                                       alpha_L0=alpha_L0[index],
                                       alpha_0=alpha_0[index],
                                       CD0=np.array([CD0[0, index], CD0[1, index], CD0[2, index]]),
                                        M=M[index])
                           for index in range(spanwise_discretisation_propeller_BEM+1)],
                 ref_point=ref_point
                 )

prop2 = PropInfo(label='Prop1',
                 prop_location=0.332,
                 nr_blades=4,
                 rot_rate=300.,
                 chord=np.array(chord, order='F'),
                 twist=np.array(twist, order='F'),
                 span=np.array(span, order='F'),
                 airfoils=[AirfoilInfo(label=f'Foil_{index}',
                                       Cl_alpha=Cl_alpha[index],
                                       alpha_L0=alpha_L0[index],
                                       alpha_0=alpha_0[index],
                                       CD0=np.array([CD0[0, index], CD0[1, index], CD0[2, index]]),
                                        M=M[index])
                           for index in range(spanwise_discretisation_propeller_BEM+1)],
                 ref_point=ref_point
                 )

parameters = ParamInfo(vinf=40.,
                       wing_aoa=0.,
                       mach_number=0.2,
                       reynolds_number=640_000,
                       speed_of_sound=333.4,
                       air_density=1.2087)


wing = WingInfo(label='PROWIM_wing',
                span=wingspan,
                chord=np.ones(spanwise_discretisation_propeller_BEM,
                              order='F')*wing_chord,
                twist=np.ones(spanwise_discretisation_propeller_BEM,
                              order='F')*wing_twist,
                thickness=np.ones(spanwise_discretisation_propeller_BEM,
                              order='F')*0.001,
                empty_weight=0.,
                CL0 = 0.283
                )


wingpropinfo = WingPropInfo(
                            spanwise_discretisation_propeller=51,
                            spanwise_discretisation_propeller_BEM=spanwise_discretisation_propeller_BEM,
                            propeller=[prop1, prop2],
                            wing=wing,
                            parameters=parameters
                            )


class PROWIMAnalysis(om.Group):
    def initialize(self):
        self.options.declare('WingPropInfo', default=WingPropInfo)

    def setup(self):
        self.add_subsystem('PropellerSlipstreamWingModel',
                           subsys=WingSlipstreamPropAnalysis(WingPropInfo=wingpropinfo))

    def configure(self):
        # Empty because we do analysis
        ...


if __name__ == '__main__':
    # === Generate numerical data ===
    CL_numerical = []
    J = np.array([0.796, 0.8960, float('nan')])
    rot_rate = (wingpropinfo.parameters.vinf/(J*2.*prop_radius)) * 2.*np.pi # in rad/s
    angles = np.arange(-8, 11, 1)
    
    prob = om.Problem()

    for index_rotational, _ in enumerate(rot_rate):
        CL_numerical_tmp = []
        for index_propeller, _ in enumerate(wingpropinfo.propeller):
            wingpropinfo.propeller[index_propeller].rot_rate = rot_rate[index_rotational]
        
        if np.isnan(rot_rate[index_rotational]):
            wingpropinfo.NO_CORRECTION=True
            wingpropinfo.NO_PROPELLER=True
        
        elif J[index_rotational]==0.796:
            wingpropinfo.NO_CORRECTION=False
            wingpropinfo.NO_PROPELLER=False
            wingpropinfo.wing.CL0 = 0.3079
            
        elif J[index_rotational]==0.8960:
            wingpropinfo.NO_CORRECTION=False
            wingpropinfo.NO_PROPELLER=False
            wingpropinfo.wing.CL0 = 0.2938
            
        else:
            wingpropinfo.NO_CORRECTION=False
            wingpropinfo.NO_PROPELLER=False
            wingpropinfo.wing.CL0 = 0.283

        for angle in angles:
            print(f'Angle of attack: {angle: ^10}')
            wingpropinfo.parameters.wing_aoa = angle
            
            prob.model = PROWIMAnalysis(WingPropInfo=wingpropinfo)
            prob.setup()
            prob.run_model()

            CL_numerical_tmp.append(prob["PropellerSlipstreamWingModel.OPENAEROSTRUCT.AS_point_0.wing_perf.CL"].tolist()[0])      
            
        CL_numerical.append(CL_numerical_tmp)

    # === Load in (experimental) validation data ===
    validation_file = os.path.join(BASE_DIR, 'data', 'PROWIM_validation_conventional.txt')
    validation_data = pd.read_csv(validation_file, delimiter=',', skiprows=22)
    
    # Validation data for J=inf (prop-off)
    n=0
    index1 = n*19
    index2 = (n+1)*19
    aoa = validation_data['AoA'][index1:index2]
    CL_Jinf = validation_data['CL'][index1:index2]
    CD_Jinf = validation_data['CD'][index1:index2]
    J_inf = validation_data['J'][index1+1]
    
    # Validation data for J=1
    n=2
    index1 = n*19
    index2 = (n+1)*19
    aoa = validation_data['AoA'][index1:index2]
    CL_J1 = validation_data['CL'][index1:index2]
    CD_J1 = validation_data['CD'][index1:index2]
    J_1 = validation_data['J'][index1+1]
    
    # Validation data for J=0.796
    n=3
    index1 = n*19
    index2 = (n+1)*19
    aoa = validation_data['AoA'][index1:index2]
    CL_J0796 = validation_data['CL'][index1:index2]
    CD_J0796 = validation_data['CD'][index1:index2]
    J_0796 = validation_data['J'][index1+1]
    
    # === Plot results ===
    plt.style.use(niceplots.get_style())
    _, ax = plt.subplots(figsize=(10, 7))
    
    ax.plot(angles, CL_numerical[0], label=r'Numerical, $J=0.796$', color='b')
    ax.plot(angles, CL_numerical[1], label=r'Numerical, $J=0.896$', color='orange')
    ax.plot(angles, CL_numerical[2], label=r'Numerical, $J=\infty$', color='grey')

    ax.scatter(aoa, CL_J0796, label=r'Experimental, $J=0.796$', color='b')
    ax.scatter(aoa, CL_J1, label=r'Experimental, $J=0.896$', color='orange')
    ax.scatter(aoa, CL_Jinf, label=r'Experimental, $J=\infty$', color='grey')
    
    
    ax.set_xlabel("Angle of Attack (deg)", fontweight='light')
    ax.set_ylabel(r"Lift Coefficient ($C_L$)", fontweight='light')
    ax.legend(fontsize='12')

    niceplots.adjust_spines(ax, outward=True)

    plt.savefig(os.path.join(BASE_DIR, 'figures', 'PROWIM_VALIDATION.png'))