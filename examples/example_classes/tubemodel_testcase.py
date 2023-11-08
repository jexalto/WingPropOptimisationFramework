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

# Now we scale the prop with a factor 10
prop_radius = 0.1185*10
ref_point = data['ref_point']
span = np.array(data['span'])*10.
twist = np.array([46.77357992, 43.98483409, 41.42356386, 39.07255909, 36.90084483, 35.18368962,
                  34.06182823, 33.05810344, 32.18348507, 31.48233448, 30.8998383,  30.35113547,
                  29.76701434, 29.00781912, 27.95217468, 26.67091045, 25.10751002, 23.70509332,
                  22.39921366, 20.29109844])
chord = np.array(data['chord'])*10

alpha_0 = data['alpha_0']
alpha_L0 = data['alpha_L0']
Cl_alpha = data['Cl_alpha']
M = data['M']

J = 1.0  # advance ratio

wing_twist = 0.
wing_chord = 2
wingspan = 20.  # m

prop_refinement = 4
num_cp = 15  # wing

spanwise_discretisation_propeller_BEM = len(span)


PROWIM_parameters = ParamInfo(vinf=40.,
                              wing_aoa=2.,  # TODO: this is a wing property
                              mach_number=0.2,
                              reynolds_number=3_500_000,
                              speed_of_sound=333.4,
                              air_density=1.2087)

wingtip_prop_1 = PropInfo(label='WingTipProp1',
                          prop_location=-10,
                          nr_blades=4,
                          rot_rate=(PROWIM_parameters.vinf /
                                    (J*2.*prop_radius)) * 2.*np.pi,  # in rad/s,
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
                          local_refinement=prop_refinement,
                          rotation_direction=-1,
                          )

wingtip_prop_2 = PropInfo(label='WingTipProp2',
                          prop_location=10,
                          nr_blades=4,
                          rot_rate=(PROWIM_parameters.vinf /
                                    (J*2.*prop_radius)) * 2.*np.pi,  # in rad/s,
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
                          local_refinement=prop_refinement,
                          rotation_direction=1,
                          )

inboard_prop_1 = PropInfo(label='InboardProp1',
                          prop_location=-5,
                          nr_blades=4,
                          rot_rate=(PROWIM_parameters.vinf /
                                    (J*2.*prop_radius)) * 2.*np.pi,  # in rad/s,
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
                          local_refinement=prop_refinement,
                          rotation_direction=-1,
                          )

inboard_prop_2 = PropInfo(label='InboardProp2',
                          prop_location=5,
                          nr_blades=4,
                          rot_rate=(PROWIM_parameters.vinf /
                                    (J*2.*prop_radius)) * 2.*np.pi,  # in rad/s,
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
                          local_refinement=prop_refinement,
                          rotation_direction=1,
                          )


optimisation_wing = WingInfo(label='PROWIM_wing',
                             span=wingspan,
                             thickness=np.ones(num_cp)*0.003,
                             chord=np.ones(num_cp,
                                           order='F')*wing_chord,
                             twist=np.ones(num_cp,
                                           order='F')*wing_twist,
                             empty_weight=10.,
                             CL0=0.283,  # if you want to do optimization set this to zero bcs otherwise OAS will return erroneous results
                             CD0=0.025,
                             fuel_mass=0,
                             )


optimisation_wingpropinfo = WingPropInfo(
    spanwise_discretisation_propeller=19,
    gamma_dphi=15,
    spanwise_discretisation_propeller_BEM=spanwise_discretisation_propeller_BEM,
    propeller=[wingtip_prop_1, wingtip_prop_2],
    wing=optimisation_wing,
    parameters=PROWIM_parameters
)
