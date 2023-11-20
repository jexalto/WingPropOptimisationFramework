# --- Built-ins ---
from pathlib import Path
import logging
import os

# --- Internal ---
from openaerostruct.geometry.utils import generate_mesh, taper, sweep
from openaerostruct.integration.aerostruct_groups import AerostructGeometry, AerostructPoint
from openaerostruct.utils.constants import grav_constant
from src.postprocessing.OAS_plots.animation import animate_oas_solutions
from examples.example_classes.PROWIM_classes import PROWIM_wingpropinfo

# --- External ---
import numpy as np
import openmdao.api as om
from openmdao.recorders.sqlite_reader import SqliteCaseReader

"""
OpenAeroStruct optimization to obtain baseline wing planform (twist, chord, sweep)
"""

def get_OAS_surface(db_name:str, wingpropinfo, num_y=15, num_x=5, symmetry=True):
    """
    creates rectanble OAS surface
    """
    wing_area = 0.2285
    span = 1.6
    taper_ratio = 0.5
    sweep_angle = 5.   # deg
    root_chord = 2 * wing_area / span / (1 + taper_ratio)

    n_twist_points = 5

    # Create a dictionary to store options about the surface
    mesh_dict = {"num_y": num_y, "num_x": num_x, "wing_type": "rect", "symmetry": symmetry, "span": span, "root_chord": root_chord}

    mesh = wingpropinfo.vlm_mesh #generate_mesh(mesh_dict)
    # apply taper and sweep
    taper(mesh, taper_ratio, symmetry=symmetry)
    sweep(mesh, sweep_angle, symmetry=symmetry)

    # roughly estimate flat-plate transition point
    Re_crit = 5e5
    v = 29
    rho = 1.225
    mu = 1.81e-5
    x_crit = Re_crit * mu / (rho * v)
    # print(x_crit / chord)
    
    database = SqliteCaseReader(db_name, pre_load=True)
    
    optimised_case = database.get_cases()[-1].outputs
    
    chord = optimised_case['DESIGNVARIABLES.chord']
    twist = optimised_case['DESIGNVARIABLES.twist']
    vel_distr = optimised_case['RETHORST.velocity_distribution']

    surface = {
        # Wing definition
        "name": "wing",  # name of the surface
        "symmetry": symmetry,  # if true, model one half of wing
        # reflected across the plane y = 0
        "S_ref_type": "wetted",  # how we compute the wing area,
        # can be 'wetted' or 'projected'
        "fem_model_type": "tube",
        "thickness_cp": np.array([0.003]),
        "chord_cp": chord,
        "twist_cp": twist,
        "mesh": mesh,
        # "taper" : taper_ratio,
        # Aerodynamic performance of the lifting surface at
        # an angle of attack of 0 (alpha=0).
        # These CL0 and CD0 values are added to the CL and CD
        # obtained from aerodynamic analysis of the surface to get
        # the total CL and CD.
        # These CL0 and CD0 values do not vary wrt alpha.
        "CL0": 0.0,  # CL of the surface at alpha=0
        "CD0": 0.015,  # CD of the surface at alpha=0
        # Airfoil properties for viscous drag calculation
        "k_lam": 0.9,  # percentage of chord with laminar
        # flow, used for viscous drag
        "t_over_c_cp": np.array([0.15]),  # thickness over chord ratio (NACA0015)
        "c_max_t": 0.303,  # chordwise location of maximum (NACA0015)
        # thickness
        "with_viscous": True,
        "with_wave": False,  # if true, compute wave drag
        # Structural values are based on aluminum 7075
        "E": 72.6e9,  # [Pa] Young's modulus of the spar
        "G": 30.0e9,  # [Pa] shear modulus of the spar
        "yield": 250.0e6 / 2.5,  # [Pa] yield stress divided by 2.5 for limiting case
        "mrho": 2.79e3,  # [kg/m^3] material density
        "fem_origin": 0.35,  # normalized chordwise location of the spar
        "wing_weight_ratio": 5.0,
        "struct_weight_relief": True,  # True to add the weight of the structure to the loads on the structure
        "distributed_fuel_weight": False,
        # Constraints
        "exact_failure_constraint": False,  # if false, use KS function
    }

    return surface, vel_distr


def setup_OAS_singlepoint_problem(db_name: str, wingpropinfo, num_y=40, num_x=2):
    # setup OAS single point model
    surface, vel_distr = get_OAS_surface(db_name=db_name,wingpropinfo=wingpropinfo, num_y=num_y, num_x=num_x, symmetry=False)

    # Create the problem and assign the model group
    prob = om.Problem(reports=False)   # TODO: report generation fails with CCD...

    # Add problem information as an independent variables component
    indep_var_comp = om.IndepVarComp()
    indep_var_comp.add_output("v", val=40., units="m/s")
    indep_var_comp.add_output("velocity_distribution", val=vel_distr, units="m/s")
    indep_var_comp.add_output("alpha", val=2.0, units="deg")
    indep_var_comp.add_output("Mach_number", val=0.2)
    indep_var_comp.add_output("re", val=3.5e5, units="1/m")
    indep_var_comp.add_output("rho", val=1.208, units="kg/m**3")
    indep_var_comp.add_output("CT", val=grav_constant * 17.0e-6, units="1/s")
    indep_var_comp.add_output("R", val=0, units="m")
    indep_var_comp.add_output("W0", val=5, units="kg")
    indep_var_comp.add_output("speed_of_sound", val=330, units="m/s")
    indep_var_comp.add_output("load_factor", val=1.0)
    indep_var_comp.add_output("empty_cg", val=np.zeros((3)), units="m")

    prob.model.add_subsystem("prob_vars", indep_var_comp, promotes=["*"])

    # design variables
    design_var_comp = om.IndepVarComp()
    design_var_comp.add_output("twist_cp", val=surface["twist_cp"], units='deg')
    design_var_comp.add_output("thickness_cp", val=surface["thickness_cp"], units='m')
    prob.model.add_subsystem('wing_design_vars', design_var_comp)

    aerostruct_group = AerostructGeometry(surface=surface)

    name = "wing"

    # Add tmp_group to the problem with the name of the surface.
    prob.model.add_subsystem(name, aerostruct_group)
    # Connect design variables
    prob.model.connect('wing_design_vars.twist_cp', name + '.twist_cp')
    prob.model.connect('wing_design_vars.thickness_cp', name + '.thickness_cp')

    point_name = "AS_cruise_point"

    # Create the aero point group and add it to the model
    AS_point = AerostructPoint(surfaces=[surface])

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

    com_name = point_name + "." + name + "_perf"
    prob.model.connect(
        name + ".local_stiff_transformed", point_name + ".coupled." + name + ".local_stiff_transformed"
    )
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
    
    prob.model.connect('velocity_distribution',
                        'AS_cruise_point.coupled.aero_states.velocity_distribution')

    # compute total weight
    weight_comp = om.ExecComp('total_weight = W0 + structural_mass', units='kg', shape=(1,))
    prob.model.add_subsystem('weight_comp', weight_comp, promotes=['total_weight', 'W0'])
    prob.model.connect(name + ".structural_mass", 'weight_comp.structural_mass')

    return prob, surface


if __name__ == '__main__':
    PROWIM_wingpropinfo.spanwise_discretisation_propeller = 21
    PROWIM_wingpropinfo.__post_init__()
    
    logging.getLogger('matplotlib.font_manager').disabled = True
    BASE_DIR = Path(__file__).parents[0]
    
    db_name_isowing = os.path.join(BASE_DIR, '..', 'optimisation', 'results', 'data_wingprop_0.db')

    prob, surface = setup_OAS_singlepoint_problem(db_name=db_name_isowing, wingpropinfo=PROWIM_wingpropinfo)

    # --- optimizer ---
    prob.driver = om.pyOptSparseDriver(optimizer='SNOPT')
    prob.driver.opt_settings['Major iterations limit'] = 300
    prob.driver.opt_settings['Major feasibility tolerance'] = 1e-6
    prob.driver.opt_settings['Major optimality tolerance'] = 1e-6

    prob.driver.opt_settings['Verify level'] = -1  # do not check gradient
    prob.driver.opt_settings['Function precision'] = 1e-10
    prob.driver.opt_settings['Nonderivative linesearch'] = 1

    # approximate Hessian
    prob.driver.opt_settings['Hessian full memory'] = 1
    prob.driver.opt_settings['Hessian frequency'] = 100

    # ----------------------------
    #   optimization problem
    # ----------------------------
    # minimize CD with CL = 0.6 constraint
    # prob.model.add_design_var("wing_design_vars.twist_cp", lower=-10.0, upper=15.0, ref=10, units='deg')   # exclude root twist
    # prob.model.add_design_var("wing.sweep", lower=0., upper=30.0, ref=10, units='deg')   # exclude root twist
    
    # prob.model.add_constraint("AS_cruise_point.wing_perf.failure", upper=0.0)
    # prob.model.add_constraint("AS_cruise_point.wing_perf.thickness_intersects", upper=0.0)

    # prob.model.add_design_var("alpha", lower=-10.0, upper=10.0)
    # prob.model.add_constraint("AS_cruise_point.wing_perf.CL", equals=0.6)
    # prob.model.add_objective("AS_cruise_point.wing_perf.CD", scaler=1e-5)

    # Set up the problem
    prob.setup(check=True)

    # linear solver used to solve UDE (also, if nonlinear_solver.linear_solver is not speficied, the following solver will be used for Newton step computation)
    prob.model.AS_cruise_point.coupled.linear_solver = om.LinearBlockGS(iprint=0, maxiter=30, assemble_jac=False)

    prob.run_model()

    # om.n2(prob)

    # print results
    # print("alpha = ", prob.get_val("alpha", units="deg"))
    # print("twist = ", prob.get_val("wing.twist_cp", units="deg"))
    # print("sweep = ", prob.get_val("wing.sweep", units="deg"))
    # print("CL = ", prob.get_val("AS_cruise_point.wing_perf.CL", units=None))
    # print("CD = ", prob.get_val("AS_cruise_point.wing_perf.CD", units=None))
    # print("S_ref = ", prob.get_val("AS_cruise_point.wing_perf.S_ref", units='m**2'))
    # print("failure:", prob.get_val("AS_cruise_point.wing_perf.failure"))   # upper=0.0. If positive, structural failure!

    # ------------------------------------
    #  get OAS solution and plot 3D wing 
    # ------------------------------------
    mesh_deformed = prob.get_val('AS_cruise_point.coupled.wing.def_mesh', units='m')   # deformed mesh, (nx, ny, 3)
    mesh_undeformed = prob.get_val('wing.mesh', units='m')   # mesh before structural deformation, (nx, ny, 3)
    sec_forces = prob.get_val('AS_cruise_point.coupled.aero_states.wing_sec_forces', units='N')   # sectional forces, (nx-1, ny-1, 3)
    vm_stress = prob.get_val('AS_cruise_point.wing_perf.vonmises', units='N/m**2')   # von Mises stress, (ny-1, 2)
    stress_margin = 1 - vm_stress / surface["yield"]   # stress margin, (ny-1, 2)
    tube_radius = prob.get_val('wing.radius', units='m').flatten()   # structure tube radius, (ny-1,)
    tube_thickness = prob.get_val('wing.thickness', units='m').flatten()   # structure tube thickness, (ny-1,)

    # reformat solutions into time history with only two time indices (same solution for both time instances)
    mesh_deformed = mesh_deformed.reshape((1,) + mesh_deformed.shape)   # (1, nx, ny, 3)
    mesh_undeformed = mesh_undeformed.reshape((1,) + mesh_undeformed.shape)  # (1, nx, ny, 3)
    sec_forces = sec_forces.reshape((1,) + sec_forces.shape)  # (1, nx-1, ny-1, 3)
    stress_margin = stress_margin.reshape((1,) + stress_margin.shape)  # (1, ny-1, 2)
    tube_radius = tube_radius.reshape((1,) + tube_radius.shape)  # (1, ny-1,)
    tube_thickness = tube_thickness.reshape((1,) + tube_thickness.shape)  # (1, ny-1,)

    fem_sol = {'radius_his': tube_radius, 'thickness_his': tube_thickness, 'stress_margin_his': stress_margin, 'fem_origin': surface['fem_origin']}

    animate_oas_solutions(1, mesh_deformed, mesh_undeformed, sec_forces, fem_sol, symmetry=surface['symmetry'])
    
    import pickle
    sol = {'mesh_deformed': mesh_deformed, 'mesh_undeformed': mesh_undeformed, 'sec_forces': sec_forces, 'fem_sol': fem_sol, 'symmetry': surface['symmetry']}
    with open('OAS_solution.pkl', 'wb') as f:
        pickle.dump(sol, f)
