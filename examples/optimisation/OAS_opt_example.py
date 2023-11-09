import numpy as np
import logging

logging.getLogger('matplotlib.font_manager').disabled = True

from openaerostruct.geometry.utils import generate_mesh

from openaerostruct.integration.aerostruct_groups import AerostructGeometry, AerostructPoint

import openmdao.api as om
from openaerostruct.utils.constants import grav_constant

### from examples.example_classes.PROWIM_classes import PROWIM_wingpropinfo

# Create a dictionary to store options about the surface
num_cp = 10
ny = 21
mesh_dict = {
                "num_y": ny,
                "num_x": 3,
                "wing_type": "rect",
                "symmetry": False,
                "span": .748*2,
                "root_chord": 0.24,
            }

# mesh = PROWIM_wingpropinfo.vlm_mesh
chord = np.ones(num_cp)

mesh = generate_mesh(mesh_dict)

surface = {
    # Wing definition
    "name": "wing",  # name of the surface
    "symmetry": False,  # if true, model one half of wing
    # reflected across the plane y = 0
    "S_ref_type": "wetted",  # how we compute the wing area,
    # can be 'wetted' or 'projected'
    "fem_model_type": "tube",
    "thickness_cp": 0.01 * np.ones(num_cp),  # [m]
    "twist_cp": np.zeros(num_cp),
    "chord_cp": np.ones(num_cp),
    "mesh": mesh,
    # Aerodynamic performance of the lifting surface at
    # an angle of attack of 0 (alpha=0).
    # These CL0 and CD0 values are added to the CL and CD
    # obtained from aerodynamic analysis of the surface to get
    # the total CL and CD.
    # These CL0 and CD0 values do not vary wrt alpha.
    "CL0": 0.,  # CL of the surface at alpha=0
    "CD0": 0.015,  # CD of the surface at alpha=0
    # Airfoil properties for viscous drag calculation
    "k_lam": 0.05,  # percentage of chord with laminar
    # flow, used for viscous drag
    "t_over_c_cp": np.array([0.15]),  # thickness over chord ratio (NACA0015)
    "c_max_t": 0.303,  # chordwise location of maximum (NACA0015)
    # thickness
    "with_viscous": True,
    "with_wave": False,  # if true, compute wave drag
    # Structural values are based on aluminum 7075
    "E": 70.0e9,  # [Pa] Young's modulus of the spar
    "G": 30.0e9,  # [Pa] shear modulus of the spar
    "yield": 500.0e6 / 2.5,  # [Pa] yield stress divided by 2.5 for limiting case
    "mrho": 3.0e3,  # [kg/m^3] material density
    "fem_origin": 0.35,  # normalized chordwise location of the spar
    "wing_weight_ratio": 2.0,
    "struct_weight_relief": True,  # True to add the weight of the structure to the loads on the structure
    "distributed_fuel_weight": False,
    # Constraints
    "exact_failure_constraint": False,  # if false, use KS function
}

# Create the problem and assign the model group
prob = om.Problem()

indep_var_comp = om.IndepVarComp()
indep_var_comp.add_output("fuel_mass", 
                          val=0., units="kg") 
indep_var_comp.add_output("v", val=40, units="m/s")
indep_var_comp.add_output("chord", val=chord, units="m")
indep_var_comp.add_output("velocity_distribution", 
                          val=np.ones(ny-1)*40, units="m/s") 
indep_var_comp.add_output("alpha", val=2.0, units="deg")
indep_var_comp.add_output("Mach_number", val=0.2)
indep_var_comp.add_output("re", val=3.4e6, units="1/m")
indep_var_comp.add_output("rho", val=1.225, units="kg/m**3")
indep_var_comp.add_output("CT", val=grav_constant * 0, units="1/s")
indep_var_comp.add_output("R", val=0, units="m")
indep_var_comp.add_output("W0", val=5, units="kg")
indep_var_comp.add_output("speed_of_sound", val=333.4, units="m/s")
indep_var_comp.add_output("load_factor", val=1.0)
indep_var_comp.add_output("empty_cg", val=np.zeros((3)), units="m")

prob.model.add_subsystem("prob_vars", indep_var_comp, promotes=["*"])

aerostruct_group = AerostructGeometry(surface=surface)

name = "wing"

# Add tmp_group to the problem with the name of the surface.
prob.model.add_subsystem(name, aerostruct_group)

point_name = "AS_point_0"

# Create the aero point group and add it to the model
AS_point = AerostructPoint(surfaces=[surface],
                           internally_connect_fuelburn=False)

prob.model.add_subsystem(
    point_name,
    AS_point,
    promotes_inputs=[
        "v",
        "alpha",
        "Mach_number",
        "re",
        "rho",
        "CT",
        "R",
        "W0",
        "speed_of_sound",
        "empty_cg",
        "load_factor",
    ],
)

prob.model.connect('velocity_distribution',
                     'AS_point_0.coupled.aero_states.velocity_distribution')

prob.model.connect('fuel_mass',
                        'AS_point_0.total_perf.L_equals_W.fuelburn')
prob.model.connect('fuel_mass',
                        'AS_point_0.total_perf.CG.fuelburn')

com_name = point_name + "." + name + "_perf"
prob.model.connect(name + ".local_stiff_transformed", point_name + ".coupled." + name + ".local_stiff_transformed")
prob.model.connect(name + ".nodes", point_name + ".coupled." + name + ".nodes")

# Connect aerodyamic mesh to coupled group mesh
prob.model.connect(name + ".mesh", point_name + ".coupled." + name + ".mesh")

# Connect performance calculation variables
prob.model.connect(name + ".radius", com_name + ".radius")
prob.model.connect(name + ".thickness", com_name + ".thickness")
prob.model.connect(name + ".nodes", com_name + ".nodes")
prob.model.connect(name + ".cg_location", point_name + "." + "total_perf." + name + "_cg_location")
prob.model.connect(name + ".structural_mass", point_name + "." + "total_perf." + name + "_structural_mass")
prob.model.connect(name + ".t_over_c", com_name + ".t_over_c")

prob.driver = om.pyOptSparseDriver()
prob.driver.options["optimizer"] = "SNOPT"
prob.driver.options["debug_print"] = ['desvars', 'nl_cons', 'objs']
prob.driver.options["print_results"] = True
prob.driver.opt_settings = {
#     "Major iterations limit": 2,
    "Major feasibility tolerance": 1.0e-7,
    "Major optimality tolerance": 1.0e-8,
    "Minor feasibility tolerance": 1.0e-7,
    "Verify level": -1,
    "Function precision": 1.0e-10,
}

recorder = om.SqliteRecorder("aerostruct.db")
prob.driver.add_recorder(recorder)
prob.driver.recording_options["record_derivatives"] = True
prob.driver.recording_options["includes"] = ["*"]

# Setup problem and add design variables, constraint, and objective
prob.model.add_design_var("wing.twist_cp", lower=-10.0, upper=10.0, units='deg')
prob.model.add_design_var("wing.geometry.chord_cp", lower=0.01, upper=10.0, units='m')   # not really "m", this is a chord scaler
prob.model.add_design_var("wing.thickness_cp", lower=0.003, upper=0.02, scaler=1e2, units='m')
prob.model.add_constraint("AS_point_0.wing_perf.failure", upper=0.0)
prob.model.add_constraint("AS_point_0.wing_perf.thickness_intersects", upper=0.0)

# Add design variables, constraisnt, and objective on the problem
# prob.model.add_design_var("alpha", lower=-10.0, upper=10.0)
prob.model.add_constraint("AS_point_0.L_equals_W", equals=0.)
# prob.model.add_constraint("AS_point_0.total_perf.L", equals=24545.68752965)
prob.model.add_objective("AS_point_0.total_perf.D", units='N', scaler=1/5.)

# Set up the problem
prob.setup(check=True)
# prob.run_model()

# Run optimization
prob.run_model()

prob.check_totals(compact_print=True,
                    show_only_incorrect=True)

quit()

print(prob["AS_point_0.L_equals_W"])
print(prob["AS_point_0.total_perf.L"])
om.n2(prob, outfile='run_CRM_before_opt.html', show_browser=False)

twist_0 = prob["wing.geometry.twist"][0].copy()
chord_0 = prob["wing.geometry.chord"][0].copy()
CL0 = prob["AS_point_0.wing_perf.Cl"].copy()

prob.setup()
prob.run_driver()

om.n2(prob, outfile='run_CRM_after_opt.html', show_browser=True)

print()
print("CL:", prob["AS_point_0.wing_perf.CL"])
print("CD:", prob["AS_point_0.wing_perf.CD"])

print(prob["wing.geometry.twist"])
print(prob["wing.geometry.chord"])
print(prob[point_name + ".wing_perf.Cl"])
# print(prob[point_name + ".wing.S_ref"])

import matplotlib.pyplot as plt
import niceplots
plt.style.use(niceplots.get_style())

_, ax = plt.subplots(figsize=(10, 7))

spanwise = np.linspace(-1, 1, len(prob['wing.geometry.chord'][0]))
ax.plot(spanwise, prob["wing.geometry.chord"][0],
        label=f'chord, optimised', color='b', linestyle='dashed')
ax.plot(spanwise, chord_0,
        label=f'chord, original', color='Orange')

ax.set_ylim(0, 1.01)
ax.legend()
niceplots.adjust_spines(ax, outward=True)

plt.savefig('./chord.png')

_, ax = plt.subplots(figsize=(10, 7))

spanwise = np.linspace(-1, 1, len(prob['wing.geometry.chord'][0]))
ax.plot(spanwise, prob["wing.geometry.twist"][0],
        label=f'twist, optimised', color='b', linestyle='dashed')
ax.plot(spanwise, twist_0,
        label=f'twist, original', color='Orange')

ax.legend()
niceplots.adjust_spines(ax, outward=True)

plt.savefig('./twist.png')

_, ax = plt.subplots(figsize=(10, 7))

Cl = prob["AS_point_0.wing_perf.Cl"]
spanwise = np.linspace(-1, 1, len(Cl))
chord = prob["wing.geometry.chord"][0]
chord_cl = [(chord[index]+chord[index+1])/2 for index in range(len(Cl))]
chord0_cl = [(chord_0[index]+chord_0[index+1])/2 for index in range(len(Cl))]
ax.plot(spanwise, Cl*chord_cl,
        label=f'Cl*c, optimised', color='b', linestyle='dashed')
ax.plot(spanwise, CL0*chord0_cl,
        label=f'Cl*c, original', color='orange')

ax.legend()
niceplots.adjust_spines(ax, outward=True)

plt.savefig('./clc.png')