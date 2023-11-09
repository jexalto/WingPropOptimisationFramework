# --- Built-ins ---
from dataclasses import dataclass
import json

# --- Internal ---
from src.utils.meshing import meshing
from openaerostruct.utils.constants import grav_constant

# --- External ---
import numpy as np


@dataclass
class ParamInfo:
    vinf: float
    wing_aoa: float
    mach_number: float
    reynolds_number: float
    speed_of_sound: float
    R: float = 0 # we fly electric
    CT: float = 0 # grav_constant * 17.0e-6
    air_density: float = 1.225


@dataclass
class AirfoilInfo:
    label: str
    Cl_alpha: float     # Cl alpha lift slope
    alpha_L0: float     # zero lift angle
    alpha_0: float      # stall angle
    M: float = 50.


@dataclass
class WingInfo:
    label: str
    span: float
    chord: np.array
    twist: np.array
    thickness: np.array
    empty_weight: float
    load_factor: float = 1.
    empty_cg: np.array = np.zeros((3))
    CL0: float = 0.
    CD0: float = 0.
    fuel_mass: float = 0. # we fly eletric
    youngsmodulus: float = 7e10
    yieldstress: float = 500e6/2.5
    G: float = 30e9
    mrho: float = 3e3


@dataclass
class PropInfo:
    label: str
    prop_location: float

    nr_blades: int
    rot_rate: float     # in rad/s
    chord: np.array
    twist: np.array
    span: np.array
    airfoils: list[AirfoilInfo]
    prop_angle: float = 0.
    pitch: float = 0.
    rotation_direction: int = 1 # 1 for cw, -11 for ccw
    
    esp: float = 5920 # W/kg

    rotation_axis: np.array = np.array([0., 0., 1.])
    ref_point: np.array = np.array([0., 0., 0.])
    hub_orientation: np.array = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    
    local_refinement: int = 2

    def __post_init__(self):
        assert len(self.chord) == len(self.span)+1, ' Chord should be defined for blade nodes, \
                                                        not control points (length of chord and twist should be one larger than span)'
        assert len(self.twist) == len(self.span)+1, ' Twist should be defined for blade nodes, \
                                                        not control points (length of chord and twist should be one larger than span)'

        self.prop_radius = np.zeros(len(self.span)+1)
        for index, ispan in enumerate(self.span):
            # TODO: this only works for a linearly spaced propeller
            self.prop_radius[index+1] = self.ref_point[1]+(index+1)*ispan # TODO: simplification (index 1 shouldn't be hardcoded)

    def __str__(self):
        return f'Propeller {self.label}, with {self.nr_blades} blades'


@dataclass
class WingPropInfo:
    spanwise_discretisation_propeller: int
    # TODO: right now it is assumed that all propellers have the same discretisation.
    #           This is not necessarily true
    spanwise_discretisation_propeller_BEM: int

    propeller: list[PropInfo]
    wing: WingInfo
    parameters: ParamInfo
    
    NO_CORRECTION: bool = False # set this to true if you want to run the system without a correction factor
    NO_PROPELLER: bool = False # Set this to true to run system without propeller or correction
    
    linear_mesh: bool = False
    
    # Parameters for tube model
    gamma_tangential_dx: float = 0.3 # make sure that this values doesn't place a vortex ring too close to a collocation point: at 75% of chord
    gamma_tangential_x: float = 1.0 # should be a few times larger than the chord length!
    gamma_dphi: int = 10 # radial discretisation of wake
    
    force: np.array = np.zeros(1)
    
    if NO_PROPELLER:
        assert (not NO_CORRECTION), 'ERROR: no propeller so no correction'

    def __post_init__(self):
        # === Propeller information ===
        self.nr_props = len(self.propeller)
        self.prop_locations = np.zeros((self.nr_props), order='F')
        self.prop_radii = np.zeros(
            (self.nr_props, self.spanwise_discretisation_propeller_BEM+1), order='F')

        #  === Merge the propeller information into a single array ===
        for index, _ in enumerate(self.prop_locations):
            self.prop_locations[index] = self.propeller[index].prop_location
            self.prop_radii[index] = self.propeller[index].prop_radius
        
        # === Meshing ===
        self._meshing()
                
        # === Misc variables ===
        self.velocity_distribution_nopropeller = np.ones((self.spanwise_discretisation_nodes-1))*self.parameters.vinf
        self.forceinput = self.force #np.ones((self.propeller[-1].local_refinement*len(self.propeller[-1].span)))*self.force
        
    def _meshing(self):
        prop_spacing = (2*self.prop_radii[0, -1])/self.spanwise_discretisation_propeller
        self.spanwise_discretisation_wing = int((self.wing.span-self.nr_props*2*self.prop_radii[0, -1])/((self.nr_props+1)*prop_spacing))
        if self.spanwise_discretisation_wing%2==0: self.spanwise_discretisation_wing+=1
        
        self.spanwise_discretisation_wing *= self.nr_props+1
        
        self.vlm_mesh = meshing(span=self.wing.span,
                                chord=self.wing.chord[0],
                                prop_locations=self.prop_locations,
                                prop_radii=self.prop_radii,
                                nr_props=self.nr_props,
                                spanwise_discretisation_wing=self.spanwise_discretisation_wing,
                                spanwise_panels_propeller=self.spanwise_discretisation_propeller)
        
        self.spanwise_discretisation_nodes = np.shape(self.vlm_mesh)[1]
        
        print('PROP VERSUS WING SPACING: ', ((self.wing.span-self.nr_props*2*self.prop_radii[0, -1])/(self.spanwise_discretisation_wing))/prop_spacing, ' PLEASE MAKE SURE THIS VLAUE IS CLOSE TO 1.0 FOR BEST RESULTS')

        if self.linear_mesh: # if you want to compare the custom mesh to a 'normal' OAS linear mesh
            from openaerostruct.geometry.utils import generate_mesh
            
            num_cp = len(self.wing.twist)
            
            mesh_dict = {"num_y": self.spanwise_discretisation_nodes,
                "num_x": 2,
                "wing_type": "rect",
                "symmetry": False,
                "span": self.wing.span,
                "root_chord": self.wing.chord[0],
                "num_twist_cp": num_cp
                }
            
            self.vlm_mesh = generate_mesh(mesh_dict)
        
        self.vlm_mesh_control_points = np.zeros(
            self.spanwise_discretisation_nodes-1, order='F')

        for panel in range(self.spanwise_discretisation_nodes-1):
            self.vlm_mesh_control_points[panel] = 0.5 * \
                (self.vlm_mesh[0, panel, 1]+self.vlm_mesh[0, panel+1, 1])
        