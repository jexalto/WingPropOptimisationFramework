# --- Built-ins ---
import os
from pathlib import Path
import json

# --- Internal ---
from src.base import ParamInfo, WingPropInfo, WingInfo, PropInfo, AirfoilInfo

# --- External ---
import numpy as np

BASE_DIR = Path(__file__).parents[1]


# === Read in PROWIM data ===
with open(os.path.join(BASE_DIR, 'analysis', 'data', 'PROWIM.json'), 'r') as file:
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

J = 1.0 # advance ratio

wing_twist = 0.
wing_chord = 0.24
wingspan = 0.73*2.*0.952

prop_refinement = 4

spanwise_discretisation_propeller_BEM = len(span)


PROWIM_parameters = ParamInfo(vinf=40.,
                       wing_aoa=2., # TODO: this is a wing property
                       mach_number=0.2,
                       reynolds_number=640_000,
                       speed_of_sound=333.4,
                       air_density=1.2087)

PROWIM_prop_1 = PropInfo(label='Prop1',
                 prop_location=-0.332,
                 nr_blades=4,
                 rot_rate=(PROWIM_parameters.vinf/(J*2.*prop_radius)) * 2.*np.pi, # in rad/s,
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
                 local_refinement=prop_refinement
                 )

PROWIM_prop_2 = PropInfo(label='Prop1',
                 prop_location=0.332,
                 nr_blades=4,
                 rot_rate=(PROWIM_parameters.vinf/(J*2.*prop_radius)) * 2.*np.pi, # in rad/s,
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
                 local_refinement=prop_refinement
                 )


PROWIM_wing = WingInfo(label='PROWIM_wing',
                span=wingspan,
                thickness = np.ones(9)*0.001,
                chord=np.ones(9,
                              order='F')*wing_chord,
                twist=np.ones(9,
                              order='F')*wing_twist,
                empty_weight=10.,
                CL0 = 0.283
                )


PROWIM_wingpropinfo = WingPropInfo(spanwise_discretisation_wing=60,
                            spanwise_discretisation_propeller=25,
                            spanwise_discretisation_propeller_BEM=spanwise_discretisation_propeller_BEM,
                            propeller=[PROWIM_prop_1, PROWIM_prop_2],
                            wing=PROWIM_wing,
                            parameters=PROWIM_parameters
                            )