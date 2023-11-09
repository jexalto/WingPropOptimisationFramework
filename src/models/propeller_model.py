# --- Built-ins ---

# --- Internal ---
from src.base import ParamInfo, PropInfo, WingPropInfo
import helix.parameters.simparam_def as py_simparam_def
import helix.references.references_def as py_ref_def
import helix.geometry.geometry_def as py_geo_def
import helix.geometry.geometry_def_parametric as py_geo_def_parametric
import helix.openmdao.om_helix as om_helix

# --- External ---
import openmdao.api as om
import numpy as np

TIME_STEPS_HELIX = 5 # timesteps taken by helix

class PropellerCoupled(om.ExplicitComponent):
    def initialize(self):
        self.options.declare('WingPropInfo', default=WingPropInfo)
        
    def setup(self):
        # === Options ===
        self.wingpropinfo = self.options["WingPropInfo"]
        
        # === Inputs ===
        for propeller_nr in range(self.wingpropinfo.nr_props):
            self.add_input(f'thrust_prop_{propeller_nr}', shape_by_conn=True)
            self.add_input(f'power_prop_{propeller_nr}', shape_by_conn=True)
                # the 'shape_by_conn=True' is not best practice but HELIX time dependent so it was necessary
            
        # === Outputs ===
        self.add_output('thrust_total', shape=1)
        self.add_output('power_total', shape=1)
        
        # === Partials ===
        for propeller_nr in range(self.wingpropinfo.nr_props):
            self.declare_partials('thrust_total', f'thrust_prop_{propeller_nr}', 
                                    rows=[0], cols=[3*TIME_STEPS_HELIX-1], val=1)
            self.declare_partials('power_total', f'power_prop_{propeller_nr}', 
                                    rows=[0], cols=[0], val=1)
        
    def compute(self, inputs, outputs):
        thrust, power = [], []
        
        for propeller_nr in range(self.wingpropinfo.nr_props):
            thrust.append(inputs[f'thrust_prop_{propeller_nr}'][2, TIME_STEPS_HELIX-1]) # only last entries contain the value
            power.append(inputs[f'power_prop_{propeller_nr}'][0])
            
        outputs['thrust_total'] = np.sum(thrust)
        outputs['power_total'] = np.sum(power)


class PropellerModel(om.Group):
    def initialize(self):
        self.options.declare('ParamInfo', default=ParamInfo)
        self.options.declare('PropInfo', default=PropInfo)
        
    def setup(self):
        # === Options ===
        self.paraminfo = self.options["ParamInfo"]
        self.propellerinfo = self.options["PropInfo"]

        # === Components ===
        simparam_def = self._simparam_definition()
        references_def = self._references_definition()
        geometry_def = self._geometry_definition()

        self.add_subsystem(
            "om_helix",
            om_helix.HELIX_Group(
                simparam_def=simparam_def,
                references_def=references_def,
                geometry_def=geometry_def,
                thrust_calc=True,
                torque_calc=True,
                moment_calc=True,
                power_calc=True,
                loads_calc=True,
                velocity_distribution_calc=True,
                force_distribution_calc=True,
            ),
        )
    
    # rst simparam
    def _simparam_definition(self):
        simparam = py_simparam_def.t_simparam_def()
        simparam.basename = "PropModel"

        simparam.nt = TIME_STEPS_HELIX
        simparam.dt = 1.0
        simparam.t_start = 0.0
        simparam.t_end = 0.1

        simparam.nt_rev = 30

        simparam.v_inf = np.array([0.0, 0.0, -self.paraminfo.vinf]) # TODO: assuming axial flow
        simparam.rho_inf = self.paraminfo.air_density

        return simparam


    # rst simparam (end)

    # rst ref
    def _references_definition(self):
        # Initialize Reference Frame Defintions Holder
        references_def = py_ref_def.t_references_def()

        # Hub Frame
        Hub = py_ref_def.t_frame_def()
        Hub.Name = "Hub"
        Hub.Parent = "Root"
        Hub.origin = np.array([0.0, 0.0, 0.0])
        Hub.orientation = self.propellerinfo.hub_orientation

        Hub.moving = False

        # Append to References
        references_def.append_frame(Hub)

        return references_def


    # rst ref (end)

    # rst geodef
    def _geometry_definition(self):
        # Initialize Geometry Component Definitions Holder
        geometry_def = py_geo_def.t_geometry_def()

        # ---------------------------- Blade Parameters -------------------------- #
        rotor = py_geo_def_parametric.t_geometry_def_parametric()
        rotor.CompName = "rotor"
        rotor.CompType = "rotor"
        rotor.RefName = "Hub"

        # Reference Parameters
        N_span = len(self.propellerinfo.span) # this defines nodes, NOT CONTROL POINTS
        rotor.ref_point = self.propellerinfo.ref_point
        rotor.ref_chord_frac = 0.5

        # Symmetry Parameters
        rotor.symmetry = False
        rotor.symmetry_point = np.array([0.0, 0.0, 0.0])
        rotor.symmetry_normal = np.array([0.0, 1.0, 0.0])

        # Mirror Parameters
        rotor.mirror = False
        rotor.mirror_point = np.array([0.0, 0.0, 0.0])
        rotor.mirror_normal = np.array([0.0, 1.0, 0.0])

        # Initialize Rotor and Allocate Arrays
        rotor.initialize_parametric_geometry_definition(N_span)

        rotor.multiple = True
        rotor.multiplicity = {
            "mult_type": "rotor",
            "n_blades": self.propellerinfo.nr_blades,
            "rot_axis": self.propellerinfo.rotation_axis,
            "rot_rate": self.propellerinfo.rot_rate,
            "psi_0": 0.0,
            "hub_offset": 0.,
            "n_dofs": 0,
        }

        # ------------------------ Blade Section Definition ---------------------- #
        for iSection in range(N_span+1):
            # This defines the properties at mesh nodes
            rotor.sec[iSection].chord = self.propellerinfo.chord[iSection]
            rotor.sec[iSection].twist = self.propellerinfo.twist[iSection]
            rotor.sec[iSection].alpha_0 = self.propellerinfo.airfoils[iSection].alpha_0
            rotor.sec[iSection].alpha_L0 = self.propellerinfo.airfoils[iSection].alpha_L0
            rotor.sec[iSection].Cl_alpha = self.propellerinfo.airfoils[iSection].Cl_alpha
            rotor.sec[iSection].M = self.propellerinfo.airfoils[iSection].M # this determine how steep the stall curve is
            rotor.sec[iSection].cd0 = self.propellerinfo.airfoils[iSection].CD0

        # Span Sections
        for iSpan in range(N_span):
            # This defines the properties at control points
            rotor.span[iSpan].span = self.propellerinfo.span[iSpan]
            rotor.span[iSpan].sweep = 0.0
            rotor.span[iSpan].dihed = 0.0
            rotor.span[iSpan].N_elem_span = self.propellerinfo.local_refinement # local refinement of spanwise blade section
            rotor.span[iSpan].span_type = "uniform"

        # Append To Vehicle
        geometry_def.append_component(rotor)

        return geometry_def
