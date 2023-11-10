# --- Built-ins ---

# --- Internal ---
from src.base import WingPropInfo, PropInfo, ParamInfo
from src.models.propeller_model import PropellerModel, PropellerCoupled
from src.models.wing_model import WingModelTube, WingModelWingBox
from src.models.slipstream_model import SlipStreamModel
from src.models.parameters import Parameters
from src.models.design_variables import DesignVariables
from src.constraints.constraints import ConstraintsThrustDrag
from src.utils.optUtils import bspline_interpolant

from slipstream.slipstream_rethorst import SlipstreamRethorst
from slipstream.slipstream_tube import SliptreamTube

# --- External ---
import numpy as np
import openmdao.api as om


class WingSlipstreamPropOptimisation(om.Group):
    def initialize(self):
        self.options.declare('WingPropInfo', default=WingPropInfo)
        self.options.declare('objective', default=dict)
        self.options.declare('constraints', default=dict)
        self.options.declare('design_vars', default=dict)

    def setup(self):
        # === Options ===
        wingpropinfo = self.options["WingPropInfo"]
        self.blade_nDVSec = 20

        # === Components ===
        # Inputs
        self.add_subsystem('PARAMETERS', subsys=Parameters(
            WingPropInfo=wingpropinfo))

        self.add_subsystem('DESIGNVARIABLES', subsys=DesignVariables(
            WingPropInfo=wingpropinfo))

        # Modules
        for propeller_nr in range(wingpropinfo.nr_props):
            self.add_subsystem(f'HELIX_{propeller_nr}',
                               subsys=PropellerModel(ParamInfo=wingpropinfo.parameters,
                                                     PropInfo=wingpropinfo.propeller[propeller_nr]))

            # Connections are given here for readability
            self.connect(f"DESIGNVARIABLES.rotor_{propeller_nr}_chord",
                         f"HELIX_{propeller_nr}.om_helix.geodef_parametric_0_chord")

        self.add_subsystem('RETHORST',
                           subsys=SlipstreamRethorst(   propeller_quantity=len(wingpropinfo.propeller),
                                                        propeller_discretisation_BEM=wingpropinfo.spanwise_discretisation_propeller_BEM,
                                                        propeller_discretisation=wingpropinfo.spanwise_discretisation_propeller,
                                                        propeller_tipradii=[wingpropinfo.propeller[index].prop_radius[-1] for index in range(wingpropinfo.nr_props)],
                                                        propeller_local_refinement=wingpropinfo.propeller[0].local_refinement,
                                                        NO_CORRECTION=wingpropinfo.NO_CORRECTION,
                                                        NO_PROPELLER=wingpropinfo.NO_PROPELLER,
                                                        mesh=wingpropinfo.vlm_mesh,))

        self.add_subsystem('HELIX_COUPLED',
                           subsys=PropellerCoupled(WingPropInfo=wingpropinfo))
        
        # Coupled group
        coupled_OAS_TUBE = om.Group()
        
        coupled_OAS_TUBE.add_subsystem('OPENAEROSTRUCT',
                                        subsys=WingModelTube(WingPropInfo=wingpropinfo))

        coupled_OAS_TUBE.add_subsystem('TUBEMODEL',
                                        subsys=SliptreamTube(   propeller_quantity=len(wingpropinfo.propeller),
                                                                wingpropinfo=wingpropinfo,
                                                                prop_rotation=[wingpropinfo.propeller[index].rotation_direction for index in range(len(wingpropinfo.propeller))],
                                                                nr_blades=[wingpropinfo.propeller[index].nr_blades for index in range(len(wingpropinfo.propeller))],
                                                                prop_angle=[wingpropinfo.propeller[index].prop_angle for index in range(len(wingpropinfo.propeller))],
                                                                prop_location=[wingpropinfo.propeller[index].prop_location for index in range(len(wingpropinfo.propeller))],
                                                                propeller_tipradii=[wingpropinfo.propeller[index].prop_radius[-1] for index in range(wingpropinfo.nr_props)],
                                                                propeller_local_refinement=wingpropinfo.propeller[0].local_refinement,
                                                                gamma_tangential_dx=wingpropinfo.gamma_tangential_dx,
                                                                gamma_dphi=wingpropinfo.gamma_dphi,
                                                                gamma_tangential_x=wingpropinfo.gamma_tangential_x,
                                                                propeller_discretisation_BEM=wingpropinfo.spanwise_discretisation_propeller_BEM,
                                                                propeller_discretisation=wingpropinfo.spanwise_discretisation_propeller,
                                                                mesh=wingpropinfo.vlm_mesh))
        
        coupled_OAS_TUBE.nonlinear_solver = om.NonlinearBlockGS(use_aitken=True)
        coupled_OAS_TUBE.nonlinear_solver.options["maxiter"] = 100
        coupled_OAS_TUBE.nonlinear_solver.options["atol"] = 1e-4 #TODO: why does it perform so bad for certain angles+advance ratios
        coupled_OAS_TUBE.nonlinear_solver.options["rtol"] = 1e-30
        coupled_OAS_TUBE.nonlinear_solver.options["iprint"] = 2
        coupled_OAS_TUBE.nonlinear_solver.options["err_on_non_converge"] = False

        coupled_OAS_TUBE.linear_solver = om.PETScKrylov()
        coupled_OAS_TUBE.linear_solver.precon = om.LinearRunOnce(iprint=-1)
        # coupled_OAS_TUBE.linear_solver.options["assemble_jac"] = True
        coupled_OAS_TUBE.linear_solver.options["atol"] = 1e-6
        coupled_OAS_TUBE.linear_solver.options["iprint"] = 1
        coupled_OAS_TUBE.linear_solver.options["err_on_non_converge"] = True
        
        self.add_subsystem('COUPLED_OAS_TUBE',
                           subsys=coupled_OAS_TUBE,
                           promotes_inputs=['*'],
                           promotes_outputs=['*'])

        # Outputs
        self.add_subsystem('CONSTRAINTS',
                           subsys=ConstraintsThrustDrag())

        # === Explicit connections ===
        # PARAMS to HELIX
        for index, _ in enumerate(wingpropinfo.propeller):
            self.connect(f"PARAMETERS.rotor_{index}_radius",
                         f"HELIX_{index}.om_helix.geodef_parametric_0_span")

        # PARAMS to RETHORST
        self.connect("PARAMETERS.vinf",
                     "RETHORST.interpolation.vinf")
        self.connect("PARAMETERS.propeller_locations",
                     "RETHORST.propeller_locations")
        self.connect("PARAMETERS.vinf",
                     "RETHORST.correction.vinf")
        self.connect("PARAMETERS.wing_mesh",
                     "RETHORST.wing_mesh")
        self.connect("PARAMETERS.wing_mesh_control_points",
                     "RETHORST.wing_mesh_control_points")

        # PARAMS to OPENAEROSTRUCT
        self.connect('PARAMETERS.vinf',
                     'OPENAEROSTRUCT.v')
        self.connect('PARAMETERS.alpha',
                     'OPENAEROSTRUCT.alpha')
        self.connect('PARAMETERS.Mach_number',
                     'OPENAEROSTRUCT.Mach_number')
        self.connect('PARAMETERS.re',
                     'OPENAEROSTRUCT.re')
        self.connect('PARAMETERS.rho',
                     'OPENAEROSTRUCT.rho')
        self.connect('PARAMETERS.CT',
                     'OPENAEROSTRUCT.CT')
        self.connect('PARAMETERS.R',
                     'OPENAEROSTRUCT.R')
        self.connect('PARAMETERS.speed_of_sound',
                     'OPENAEROSTRUCT.speed_of_sound')
        self.connect('PARAMETERS.load_factor',
                     'OPENAEROSTRUCT.load_factor')
        self.connect('PARAMETERS.empty_cg',
                     'OPENAEROSTRUCT.empty_cg')
        self.connect('PARAMETERS.fuel_mass',
                     'OPENAEROSTRUCT.AS_point_0.total_perf.L_equals_W.fuelburn')
        self.connect('PARAMETERS.fuel_mass',
                     'OPENAEROSTRUCT.AS_point_0.total_perf.CG.fuelburn')
        
        # HELIX_COUPLED to OPENAEROSTRUCT
        self.connect('HELIX_COUPLED.weight_total',
                     'OPENAEROSTRUCT.W0')
        
        # PARAMS to TUBEMODEL
        for index, _ in enumerate(wingpropinfo.propeller):
            self.connect("PARAMETERS.vinf",
                        f"TUBEMODEL.TUBEMODEL_coupled.TUBEMODEL_circulations_{index}.vinf")
            self.connect("PARAMETERS.vinf",
                        f"TUBEMODEL.TUBEMODEL_coupled.TUBEMODEL_KuttaJoukowski_{index}.vinf")
            self.connect("PARAMETERS.rho",
                        f"TUBEMODEL.TUBEMODEL_coupled.TUBEMODEL_KuttaJoukowski_{index}.rhoinf")

        # DVs to TUBEMODEL
        for index, _ in enumerate(wingpropinfo.propeller):
            self.connect(f"DESIGNVARIABLES.rotor_{index}_rot_rate",
                         f"TUBEMODEL.TUBEMODEL_coupled.TUBEMODEL_circulations_{index}.propeller_omega")
            self.connect(f"DESIGNVARIABLES.rotor_{index}_rot_rate",
                         f"TUBEMODEL.TUBEMODEL_coupled.TUBEMODEL_KuttaJoukowski_{index}.propeller_omega")
        
        # DVs to HELIX
        for index, _ in enumerate(wingpropinfo.propeller):
            self.connect(f"DESIGNVARIABLES.rotor_{index}_twist",
                         f"HELIX_{index}.om_helix.geodef_parametric_0_twist")
            self.connect(f"DESIGNVARIABLES.rotor_{index}_rot_rate",
                         f"HELIX_{index}.om_helix.geodef_parametric_0_rot_rate")

        # DVs to OPENAEROSTRUCT
        self.connect('DESIGNVARIABLES.twist',
                     'OPENAEROSTRUCT.wing.twist_cp')
        self.connect('DESIGNVARIABLES.chord',
                     'OPENAEROSTRUCT.wing.geometry.chord_cp')

        # HELIX to SLIPSTREAM
        for index in range(wingpropinfo.nr_props):
            # Rethorst
            self.connect(f"HELIX_{index}.om_helix.rotorcomp_0_radii",
                         f"RETHORST.interpolation.propeller_radii_BEM_rotor{index}")
            self.connect(f"HELIX_{index}.om_helix.rotorcomp_0_velocity_distribution",
                         f"RETHORST.interpolation.propeller_velocity_BEM_rotor{index}")
            # Tube model
            self.connect(f"HELIX_{index}.om_helix.rotorcomp_0_radii",
                         f"TUBEMODEL.TUBEMODEL_coupled.TUBEMODEL_forceinterpolation_{index}.propeller_radii_BEM_rotor0")
            self.connect(f"HELIX_{index}.om_helix.rotorcomp_0_f_a", # f"HELIX_{index}.om_helix.rotorcomp_0_f_a", # PARAMETERS.force_distr #
                         f"TUBEMODEL.TUBEMODEL_coupled.TUBEMODEL_forceinterpolation_{index}.propeller_force_BEM_rotor0")

        # HELIX to HELIX_COUPLED
        for index in range(wingpropinfo.nr_props):
            self.connect(f"HELIX_{index}.om_helix.rotorcomp_0_thrust",
                         f"HELIX_COUPLED.thrust_prop_{index}")
            self.connect(f"HELIX_{index}.om_helix.rotorcomp_0_power",
                         f"HELIX_COUPLED.power_prop_{index}")

        # RETHORST to OPENAEROSTRUCT
        self.connect("RETHORST.velocity_distribution",
                     "OPENAEROSTRUCT.AS_point_0.coupled.aero_states.velocity_distribution")
        self.connect("RETHORST.correction_matrix",
                     "OPENAEROSTRUCT.AS_point_0.coupled.aero_states.rethorst_correction")

        # OPENAEROSTRUCT to TUBEMODEL
        for index in range(wingpropinfo.nr_props):
            self.connect('OPENAEROSTRUCT.AS_point_0.coupled.aero_states.coll_pts',
                        f'TUBEMODEL.TUBEMODEL_coupled.TUBEMODEL_biotsavart_{index}.collocation_points')
        
        # TUBEMODEL to OPENAEROSTRUCT
        self.connect('TUBEMODEL.TUBEMODEL_velocity_output.velocity_vector',
                     'OPENAEROSTRUCT.AS_point_0.coupled.aero_states.induced_velocity_vector')
        
        # OPENAEROSTRUCT to CONSTRAINTS
        self.connect('OPENAEROSTRUCT.AS_point_0.total_perf.D',
                     'CONSTRAINTS.drag_total')

        # HELIX_COUPLED to CONSTRAINTS
        self.connect('HELIX_COUPLED.thrust_total',
                     'CONSTRAINTS.thrust_total')

    def configure(self):
        # === Options ===
        objective = self.options['objective']
        constraints = self.options['constraints']
        design_vars = self.options['design_vars']

        # === Add design variables ===
        for design_var_key in design_vars.keys():
            self.add_design_var(design_var_key,
                                lower=design_vars[design_var_key]['lb'],
                                upper=design_vars[design_var_key]['ub'],
                                scaler=design_vars[design_var_key]['scaler'])

        # === Add constraints ===
        for constraints_key in constraints.keys():
            for subkey in constraints[constraints_key].keys():
                if len(constraints[constraints_key].keys()) == 1:
                    if subkey == 'equals':
                        self.add_constraint(constraints_key,
                                            equals=constraints[constraints_key]['equals'])
                    elif subkey == 'upper':
                        self.add_constraint(constraints_key,
                                            upper=constraints[constraints_key]['upper'])
                    elif subkey == 'lower':
                        self.add_constraint(constraints_key,
                                            lower=constraints[constraints_key]['lower'])

                elif len(constraints[constraints_key].keys()) == 2:
                    self.add_constraint(constraints_key,
                                        lower=constraints[constraints_key]['lower'],
                                        upper=constraints[constraints_key]['upper'])
                    break  # TODO: there's a better way to solve this issue

        # === Add objective ===
        for objective_key in objective.keys():
            self.add_objective(objective_key,
                               scaler=objective[objective_key]['scaler'])

class WingRethorstPropOptimisation(om.Group):
    def initialize(self):
        self.options.declare('WingPropInfo', default=WingPropInfo)
        self.options.declare('objective', default=dict)
        self.options.declare('constraints', default=dict)
        self.options.declare('design_vars', default=dict)

    def setup(self):
        # === Options ===
        wingpropinfo = self.options["WingPropInfo"]
        self.blade_nDVSec = 20

        # === Components ===
        # Inputs
        self.add_subsystem('PARAMETERS', subsys=Parameters(
            WingPropInfo=wingpropinfo))

        self.add_subsystem('DESIGNVARIABLES', subsys=DesignVariables(
            WingPropInfo=wingpropinfo))

        # Modules
        for propeller_nr in range(wingpropinfo.nr_props):
            self.add_subsystem(f'HELIX_{propeller_nr}',
                               subsys=PropellerModel(ParamInfo=wingpropinfo.parameters,
                                                     PropInfo=wingpropinfo.propeller[propeller_nr]))

            # Connections are given here for readability
            self.connect(f"DESIGNVARIABLES.rotor_{propeller_nr}_chord",
                         f"HELIX_{propeller_nr}.om_helix.geodef_parametric_0_chord")

        self.add_subsystem('RETHORST',
                           subsys=SlipstreamRethorst(propeller_quantity=len(wingpropinfo.propeller),
                                                    propeller_discretisation_BEM=wingpropinfo.spanwise_discretisation_propeller_BEM,
                                                    propeller_discretisation=wingpropinfo.spanwise_discretisation_propeller,
                                                    propeller_tipradii=[wingpropinfo.propeller[index].prop_radius[-1] for index in range(wingpropinfo.nr_props)],
                                                    propeller_local_refinement=wingpropinfo.propeller[0].local_refinement,
                                                    NO_CORRECTION=wingpropinfo.NO_CORRECTION,
                                                    NO_PROPELLER=wingpropinfo.NO_PROPELLER,
                                                    mesh=wingpropinfo.vlm_mesh,))

        self.add_subsystem('OPENAEROSTRUCT',
                           subsys=WingModelTube(WingPropInfo=wingpropinfo))

        self.add_subsystem('HELIX_COUPLED',
                           subsys=PropellerCoupled(WingPropInfo=wingpropinfo))

        # Outputs
        self.add_subsystem('CONSTRAINTS',
                           subsys=ConstraintsThrustDrag())

        # === Explicit connections ===
        # PARAMS to HELIX
        for index, _ in enumerate(wingpropinfo.propeller):
            self.connect(f"PARAMETERS.rotor_{index}_radius",
                         f"HELIX_{index}.om_helix.geodef_parametric_0_span")

        # PARAMS to RETHORST
        self.connect("PARAMETERS.vinf",
                     "RETHORST.interpolation.vinf")
        self.connect("PARAMETERS.propeller_locations",
                     "RETHORST.propeller_locations")
        self.connect("PARAMETERS.vinf",
                     "RETHORST.correction.vinf")
        self.connect("PARAMETERS.wing_mesh",
                     "RETHORST.wing_mesh")
        self.connect("PARAMETERS.wing_mesh_control_points",
                     "RETHORST.wing_mesh_control_points")

        # PARAMS to OPENAEROSTRUCT
        self.connect('PARAMETERS.vinf',
                     'OPENAEROSTRUCT.v')
        self.connect('PARAMETERS.alpha',
                     'OPENAEROSTRUCT.alpha')
        self.connect('PARAMETERS.Mach_number',
                     'OPENAEROSTRUCT.Mach_number')
        self.connect('PARAMETERS.re',
                     'OPENAEROSTRUCT.re')
        self.connect('PARAMETERS.rho',
                     'OPENAEROSTRUCT.rho')
        self.connect('PARAMETERS.CT',
                     'OPENAEROSTRUCT.CT')
        self.connect('PARAMETERS.R',
                     'OPENAEROSTRUCT.R')
        self.connect('PARAMETERS.speed_of_sound',
                     'OPENAEROSTRUCT.speed_of_sound')
        self.connect('PARAMETERS.load_factor',
                     'OPENAEROSTRUCT.load_factor')
        self.connect('PARAMETERS.empty_cg',
                     'OPENAEROSTRUCT.empty_cg')
        self.connect('PARAMETERS.fuel_mass',
                     'OPENAEROSTRUCT.AS_point_0.total_perf.L_equals_W.fuelburn')
        self.connect('PARAMETERS.fuel_mass',
                     'OPENAEROSTRUCT.AS_point_0.total_perf.CG.fuelburn')
        
        # HELIX_COUPLED to OPENAEROSTRUCT
        self.connect('HELIX_COUPLED.weight_total',
                     'OPENAEROSTRUCT.W0')

        # DVs to HELIX
        for index, _ in enumerate(wingpropinfo.propeller):
            self.connect(f"DESIGNVARIABLES.rotor_{index}_twist",
                         f"HELIX_{index}.om_helix.geodef_parametric_0_twist")
            self.connect(f"DESIGNVARIABLES.rotor_{index}_rot_rate",
                         f"HELIX_{index}.om_helix.geodef_parametric_0_rot_rate")

        # DVs to OPENAEROSTRUCT
        self.connect('DESIGNVARIABLES.twist',
                     'OPENAEROSTRUCT.wing.twist_cp')
        self.connect('DESIGNVARIABLES.chord',
                     'OPENAEROSTRUCT.wing.geometry.chord_cp')
        # self.connect('DESIGNVARIABLES.span',
        #              'OPENAEROSTRUCT.wing.geometry.span')

        # HELIX to RETHORST
        for index, _ in enumerate(wingpropinfo.propeller):
            self.connect(f"HELIX_{index}.om_helix.rotorcomp_0_radii",
                        f"RETHORST.interpolation.propeller_radii_BEM_rotor{index}")
            self.connect(f"HELIX_{index}.om_helix.rotorcomp_0_velocity_distribution",
                        f"RETHORST.interpolation.propeller_velocity_BEM_rotor{index}")

        # RETHORST to OPENAEROSTRUCT
        self.connect("RETHORST.velocity_distribution",
                     "OPENAEROSTRUCT.AS_point_0.coupled.aero_states.velocity_distribution")
        self.connect("RETHORST.correction_matrix",
                     "OPENAEROSTRUCT.AS_point_0.coupled.aero_states.rethorst_correction")
        
        for index, _ in enumerate(wingpropinfo.propeller):
            self.connect(f"HELIX_{index}.om_helix.rotorcomp_0_thrust",
                        f"HELIX_COUPLED.thrust_prop_{index}")
            self.connect(f"HELIX_{index}.om_helix.rotorcomp_0_power",
                        f"HELIX_COUPLED.power_prop_{index}") 

    def configure(self):
        # === Options ===
        objective = self.options['objective']
        constraints = self.options['constraints']
        design_vars = self.options['design_vars']

        # === Add design variables ===
        for design_var_key in design_vars.keys():
            self.add_design_var(design_var_key,
                                lower=design_vars[design_var_key]['lb'],
                                upper=design_vars[design_var_key]['ub'],
                                scaler=design_vars[design_var_key]['scaler'])

        # === Add constraints ===
        for constraints_key in constraints.keys():
            for subkey in constraints[constraints_key].keys():
                if len(constraints[constraints_key].keys()) == 1:
                    if subkey == 'equals':
                        self.add_constraint(constraints_key,
                                            equals=constraints[constraints_key]['equals'])
                    elif subkey == 'upper':
                        self.add_constraint(constraints_key,
                                            upper=constraints[constraints_key]['upper'])
                    elif subkey == 'lower':
                        self.add_constraint(constraints_key,
                                            lower=constraints[constraints_key]['lower'])

                elif len(constraints[constraints_key].keys()) == 2:
                    self.add_constraint(constraints_key,
                                        lower=constraints[constraints_key]['lower'],
                                        upper=constraints[constraints_key]['upper'])
                    break  # TODO: there's a better way to solve this issue

        # === Add objective ===
        for objective_key in objective.keys():
            self.add_objective(objective_key,
                               scaler=objective[objective_key]['scaler'])


class WingOptimisation(om.Group):
    def initialize(self):
        self.options.declare('WingPropInfo', default=WingPropInfo)
        self.options.declare('objective', default=dict)
        self.options.declare('constraints', default=dict)
        self.options.declare('design_vars', default=dict)

    def setup(self):
        # === Options ===
        wingpropinfo = self.options['WingPropInfo']

        # === Add subsystems ===
        self.add_subsystem('DESIGNVARIABLES', subsys=DesignVariables(
            WingPropInfo=wingpropinfo))

        self.add_subsystem('PARAMETERS', subsys=Parameters(
            WingPropInfo=wingpropinfo))

        self.add_subsystem('OPENAEROSTRUCT',
                           subsys=WingModelTube(WingPropInfo=wingpropinfo))

        # === Connections ===
        # DVs to OPENAEROSTRUCT
        self.connect('DESIGNVARIABLES.twist',
                     'OPENAEROSTRUCT.wing.twist_cp')
        self.connect('DESIGNVARIABLES.chord',
                     'OPENAEROSTRUCT.wing.geometry.chord_cp')

        # PARAMETERS to OPENAEROSTRUCT
        self.connect('PARAMETERS.vinf',
                     'OPENAEROSTRUCT.v')
        self.connect('PARAMETERS.velocity_distribution',
                     'OPENAEROSTRUCT.AS_point_0.coupled.aero_states.velocity_distribution')
        self.connect('PARAMETERS.alpha',
                     'OPENAEROSTRUCT.alpha')
        self.connect('PARAMETERS.Mach_number',
                     'OPENAEROSTRUCT.Mach_number')
        self.connect('PARAMETERS.re',
                     'OPENAEROSTRUCT.re')
        self.connect('PARAMETERS.rho',
                     'OPENAEROSTRUCT.rho')
        self.connect('PARAMETERS.CT',
                     'OPENAEROSTRUCT.CT')
        self.connect('PARAMETERS.R',
                     'OPENAEROSTRUCT.R')
        self.connect('PARAMETERS.W0',
                     'OPENAEROSTRUCT.W0')
        self.connect('PARAMETERS.speed_of_sound',
                     'OPENAEROSTRUCT.speed_of_sound')
        self.connect('PARAMETERS.load_factor',
                     'OPENAEROSTRUCT.load_factor')
        self.connect('PARAMETERS.empty_cg',
                     'OPENAEROSTRUCT.empty_cg')
        self.connect('PARAMETERS.fuel_mass',
                     'OPENAEROSTRUCT.AS_point_0.total_perf.L_equals_W.fuelburn')
        self.connect('PARAMETERS.fuel_mass',
                     'OPENAEROSTRUCT.AS_point_0.total_perf.CG.fuelburn')

    def configure(self):
        # === Options ===
        objective = self.options['objective']
        constraints = self.options['constraints']
        design_vars = self.options['design_vars']

        # === Add design variables ===
        for design_var_key in design_vars.keys():
            self.add_design_var(design_var_key,
                                lower=design_vars[design_var_key]['lb'],
                                upper=design_vars[design_var_key]['ub'],
                                scaler=design_vars[design_var_key]['scaler'])

        # === Add constraints ===
        for constraints_key in constraints.keys():
            for subkey in constraints[constraints_key].keys():
                if len(constraints[constraints_key].keys()) == 1:
                    if subkey == 'equals':
                        self.add_constraint(constraints_key,
                                            equals=constraints[constraints_key]['equals'])
                    elif subkey == 'upper':
                        self.add_constraint(constraints_key,
                                            upper=constraints[constraints_key]['upper'])
                    elif subkey == 'lower':
                        self.add_constraint(constraints_key,
                                            lower=constraints[constraints_key]['lower'])

                elif len(constraints[constraints_key].keys()) == 2:
                    self.add_constraint(constraints_key,
                                        lower=constraints[constraints_key]['lower'],
                                        upper=constraints[constraints_key]['upper'])
                    break  # We break because otherwise it will loop over the subkey
                    # TODO: horrendous coding convention, there's a better way to solve this issue

        # === Add objective ===
        for objective_key in objective.keys():
            self.add_objective(objective_key,
                               scaler=objective[objective_key]['scaler'])

# TODO: this class seems superfluous, correct?


class PropOptimisation(om.Group):
    def initialize(self):
        self.options.declare('WingPropInfo', default=WingPropInfo)
        self.options.declare('objective', default=dict)
        self.options.declare('constraints', default=dict)
        self.options.declare('design_vars', default=dict)

    def setup(self):
        # === Options ===
        wingpropinfo = self.options['WingPropInfo']

        self.blade_nDVSec = 20  # TODO: magic number

        # === Modules ===
        self.add_subsystem('DESIGNVARIABLES', subsys=DesignVariables(
            WingPropInfo=wingpropinfo))

        for propeller_nr, _ in enumerate(wingpropinfo.propeller):
            self.add_subsystem(
                f"blade_chord_spline_{propeller_nr}",
                bspline_interpolant(
                    s=np.linspace(0, 1, self.blade_nDVSec),
                    x=np.linspace(0, 1, 20),
                    order=3,
                    deriv_1=False, deriv_2=True
                ),
            )

            self.add_subsystem(f'HELIX_{propeller_nr}',
                               subsys=PropellerModel(ParamInfo=wingpropinfo.parameters,
                                                     PropInfo=wingpropinfo.propeller[propeller_nr]))

            # Connections are given here for readability
            self.connect(f"DESIGNVARIABLES.rotor_{propeller_nr}_chord",
                         f"blade_chord_spline_{propeller_nr}.ctl_pts")
            self.connect(f"blade_chord_spline_{propeller_nr}.y",
                         f"HELIX_{propeller_nr}.om_helix.geodef_parametric_0_chord")

        self.add_subsystem('HELIX_COUPLED',
                           subsys=PropellerCoupled(WingPropInfo=wingpropinfo))

        # HELIX to HELIX_COUPLED
        for index in range(wingpropinfo.nr_props):
            self.connect(f"HELIX_{propeller_nr}.om_helix.rotorcomp_0_thrust",
                         f"HELIX_COUPLED.thrust_prop_{index}")
            self.connect(f"HELIX_{propeller_nr}.om_helix.rotorcomp_0_power",
                         f"HELIX_COUPLED.power_prop_{index}")

    def configure(self):
        # === Options ===
        wingpropinfo = self.options['WingPropInfo']
        objective = self.options['objective']
        constraints = self.options['constraints']
        design_vars = self.options['design_vars']
        chord_included = False

        # === Add design variables ===
        for design_var_key in design_vars.keys():
            self.add_design_var(design_var_key,
                                lower=design_vars[design_var_key]['lb'],
                                upper=design_vars[design_var_key]['ub'],
                                scaler=design_vars[design_var_key]['scaler'])
            if 'chord' in design_var_key:
                chord_included = True

        # === Add constraints ===
        for constraints_key in constraints.keys():
            for subkey in constraints[constraints_key].keys():
                if len(constraints[constraints_key].keys()) == 1:
                    if subkey == 'equals':
                        self.add_constraint(constraints_key,
                                            equals=constraints[constraints_key]['equals'])
                    elif subkey == 'upper':
                        self.add_constraint(constraints_key,
                                            upper=constraints[constraints_key]['upper'])
                    elif subkey == 'lower':
                        self.add_constraint(constraints_key,
                                            lower=constraints[constraints_key]['lower'])

                elif len(constraints[constraints_key].keys()) == 2:
                    self.add_constraint(constraints_key,
                                        lower=constraints[constraints_key]['lower'],
                                        upper=constraints[constraints_key]['upper'])
                    break  # TODO: there's a better way to solve this issue

        # === Additional non-adjustable constraints ===
        if chord_included:
            for propeller_nr, _ in enumerate(wingpropinfo.propeller):
                self.add_constraint(
                    f"blade_chord_spline_{propeller_nr}.d2y", upper=0.0)
                self.add_constraint(
                    f"blade_chord_spline_{propeller_nr}.y", equals=wingpropinfo.propeller[propeller_nr].chord[0], indices=[0], scaler=100.0, alias="chord_root"
                )
                self.add_constraint(
                    f"blade_chord_spline_{propeller_nr}.y", lower=0.001, upper=0.05, scaler=100.0, indices=range(1, 20), alias="chord_span"
                )

        # === Add objective ===
        for objective_key in objective.keys():
            self.add_objective(objective_key,
                               scaler=objective[objective_key]['scaler'])
            
class PropOptimisationPitch(om.Group):
    def initialize(self):
        self.options.declare('WingPropInfo', default=WingPropInfo)
        self.options.declare('objective', default=dict)
        self.options.declare('constraints', default=dict)
        self.options.declare('design_vars', default=dict)

    def setup(self):
        # === Options ===
        wingpropinfo = self.options['WingPropInfo']

        self.blade_nDVSec = 20  # TODO: magic number

        # === Modules ===
        self.add_subsystem('DESIGNVARIABLES', subsys=DesignVariables(
            WingPropInfo=wingpropinfo))

        for propeller_nr, _ in enumerate(wingpropinfo.propeller):
            self.add_subsystem(
                f"blade_chord_spline_{propeller_nr}",
                bspline_interpolant(
                    s=np.linspace(0, 1, self.blade_nDVSec),
                    x=np.linspace(0, 1, 20),
                    order=3,
                    deriv_1=False, deriv_2=True
                ),
            )

            self.add_subsystem(f'HELIX_{propeller_nr}',
                               subsys=PropellerModel(ParamInfo=wingpropinfo.parameters,
                                                     PropInfo=wingpropinfo.propeller[propeller_nr]))

            # Connections are given here for readability
            self.connect(f"DESIGNVARIABLES.rotor_{propeller_nr}_chord",
                         f"blade_chord_spline_{propeller_nr}.ctl_pts")
            self.connect(f"blade_chord_spline_{propeller_nr}.y",
                         f"HELIX_{propeller_nr}.om_helix.geodef_parametric_0_chord")
            
        # self.add_subsystem('HELIX_PITCH',
        #                    subsys=PropellerPitch(WingPropInfo=wingpropinfo))

        self.add_subsystem('HELIX_COUPLED',
                           subsys=PropellerCoupled(WingPropInfo=wingpropinfo))

        # DESIGNVARIABLES to HELIX_PITCH
        for index in range(wingpropinfo.nr_props):
            self.connect(f'DESIGNVARIABLES.rotor_{index}_twist',
                         f'HELIX_PITCH.rotor_{index}_twist')
            self.connect(f'DESIGNVARIABLES.rotor_{index}_pitch',
                         f'HELIX_PITCH.rotor_{index}_pitch')
        
        # HELIX_PITCH to HELIX
        for index in range(wingpropinfo.nr_props):
            self.connect(f'HELIX_PITCH.rotor_{index}_twist_adjusted',
                         f'HELIX_{index}.om_helix.geodef_parametric_0_twist')
        
        # HELIX to HELIX_COUPLED
        for index in range(wingpropinfo.nr_props):
            self.connect(f"HELIX_{propeller_nr}.om_helix.rotorcomp_0_thrust",
                         f"HELIX_COUPLED.thrust_prop_{index}")
            self.connect(f"HELIX_{propeller_nr}.om_helix.rotorcomp_0_power",
                         f"HELIX_COUPLED.power_prop_{index}")

    def configure(self):
        # === Options ===
        wingpropinfo = self.options['WingPropInfo']
        objective = self.options['objective']
        constraints = self.options['constraints']
        design_vars = self.options['design_vars']
        chord_included = False

        # === Add design variables ===
        for design_var_key in design_vars.keys():
            self.add_design_var(design_var_key,
                                lower=design_vars[design_var_key]['lb'],
                                upper=design_vars[design_var_key]['ub'],
                                scaler=design_vars[design_var_key]['scaler'])
            if 'chord' in design_var_key:
                chord_included = True

        # === Add constraints ===
        for constraints_key in constraints.keys():
            for subkey in constraints[constraints_key].keys():
                if len(constraints[constraints_key].keys()) == 1:
                    if subkey == 'equals':
                        self.add_constraint(constraints_key,
                                            equals=constraints[constraints_key]['equals'])
                    elif subkey == 'upper':
                        self.add_constraint(constraints_key,
                                            upper=constraints[constraints_key]['upper'])
                    elif subkey == 'lower':
                        self.add_constraint(constraints_key,
                                            lower=constraints[constraints_key]['lower'])

                elif len(constraints[constraints_key].keys()) == 2:
                    self.add_constraint(constraints_key,
                                        lower=constraints[constraints_key]['lower'],
                                        upper=constraints[constraints_key]['upper'])
                    break  # TODO: there's a better way to solve this issue

        # === Additional non-adjustable constraints ===
        if chord_included:
            for propeller_nr, _ in enumerate(wingpropinfo.propeller):
                self.add_constraint(
                    f"blade_chord_spline_{propeller_nr}.d2y", upper=0.0)
                self.add_constraint(
                    f"blade_chord_spline_{propeller_nr}.y", equals=wingpropinfo.propeller[propeller_nr].chord[0], indices=[0], scaler=100.0, alias="chord_root"
                )
                self.add_constraint(
                    f"blade_chord_spline_{propeller_nr}.y", lower=0.001, upper=0.05, scaler=100.0, indices=range(1, 20), alias="chord_span"
                )

        # === Add objective ===
        for objective_key in objective.keys():
            self.add_objective(objective_key,
                               scaler=objective[objective_key]['scaler'])
