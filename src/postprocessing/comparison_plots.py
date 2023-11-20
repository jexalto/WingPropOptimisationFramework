# --- Built-ins ---
import os
import logging

# --- Internal ---
from src.base import WingPropInfo
from src.postprocessing.utils.plotting_utils import get_niceColors, \
                                                    get_delftColors, \
                                                    get_SuperNiceColors, \
                                                    prop_circle
from examples.example_classes.PROWIM_classes import PROWIM_wingpropinfo
from openaerostruct.geometry.utils import generate_mesh

# --- External ---
import matplotlib.pyplot as plt
import niceplots
from openmdao.recorders.sqlite_reader import SqliteCaseReader
import numpy as np

def comaprison_wing(    db_name_isowing: SqliteCaseReader,
                        db_name_isoprop: SqliteCaseReader,
                        db_name_coupled: SqliteCaseReader,
                        db_name_trim: SqliteCaseReader,
                        wingpropinfo: WingPropInfo,
                        savedir: str,
                        noprop=False)->None:

    # === Misc variables ===
    span = wingpropinfo.wing.span

    # === Objective, constraints and DVs ===
    # Original
    first_case_isoprop = db_name_isoprop.get_cases()[0]
    first_case_isowing = db_name_isowing.get_cases()[0]
    first_case_coupled = db_name_coupled.get_cases()[0]

    # Optimised
    last_case_isoprop = db_name_isoprop.get_cases()[-1]
    last_case_isowing = db_name_isowing.get_cases()[-1]
    last_case_coupled = db_name_coupled.get_cases()[-1]

    # === Misc variables ===
    spanwise_mesh = wingpropinfo.vlm_mesh_control_points
    vlm_mesh = wingpropinfo.vlm_mesh[0, :, 1]
    
    vlmmesh_ctr_pnts = [0.5*(vlm_mesh[index]+vlm_mesh[index+1]) for index in range(len(vlm_mesh)-1)]
    
    # === Wing plotting ===
    Cl_wing_orig_isowing = first_case_isowing.outputs['OPENAEROSTRUCT.AS_point_0.wing_perf.Cl']
    Cl_wing_opt_isowing = last_case_isowing.outputs['OPENAEROSTRUCT.AS_point_0.wing_perf.Cl']
    Cl_wing_opt_coupled = last_case_coupled.outputs['OPENAEROSTRUCT.AS_point_0.wing_perf.Cl']
    
    # Correct the CL for incorrect normalisation in OAS
    vinf = 40.
    v_distr = last_case_coupled.outputs['RETHORST.velocity_distribution']
    v_distr_ave = np.average(v_distr)
    v_ratio = vinf**2/v_distr_ave**2

    Cl_wing_opt_coupled = np.multiply(Cl_wing_opt_coupled,
                                        v_ratio)

    try:
        chord_orig = first_case_isowing.outputs['OPENAEROSTRUCT.wing.geometry.chord'][0]*0.24
        chord_opt_isowing = last_case_isowing.outputs['OPENAEROSTRUCT.wing.geometry.chord'][0]*0.24
        chord_opt_coupled = last_case_coupled.outputs['OPENAEROSTRUCT.wing.geometry.chord'][0]*0.24
        
        chord_nodes_orig = np.zeros(len(Cl_wing_opt_isowing))
        chord_nodes_orig = [(chord_orig[index]+chord_orig[index+1])/2 for index in range(len(chord_orig)-1)]
        
        chord_nodes_opt = np.zeros(len(Cl_wing_opt_isowing))
        chord_nodes_opt = [(chord_opt_isowing[index]+chord_opt_isowing[index+1])/2 for index in range(len(chord_opt_isowing)-1)]
        
        chord_opt_coupled = [(chord_opt_coupled[index]+chord_opt_coupled[index+1])/2 for index in range(len(chord_opt_coupled)-1)]

    except:
        chord_nodes_orig = np.ones(len(Cl_wing_opt_isowing))
        chord_nodes_opt = np.ones(len(Cl_wing_opt_isowing))

    Clc_wing_orig = Cl_wing_orig_isowing*chord_nodes_orig
    Clc_wing_coupled_opt = Cl_wing_opt_coupled*chord_opt_coupled
    Clc_wing_iso_opt = Cl_wing_opt_isowing*chord_nodes_opt
    
    num_cp = 5
    mesh_dict = {"num_y": len(Clc_wing_orig)+1,
        "num_x": 2,
        "wing_type": "rect",
        "symmetry": False,
        "span": wingpropinfo.wing.span,
        "root_chord": wingpropinfo.wing.chord[0],
        "num_twist_cp": num_cp
        }
    
    isowingmesh = generate_mesh(mesh_dict)[0, :, 1]
    isowingmesh_ctr_pnts = [0.5*(isowingmesh[index]+isowingmesh[index+1]) for index in range(len(isowingmesh)-1)]

    # Area_ell = 2*sum(Clc_wing_coupled_opt*(vlm_mesh[1:]-vlm_mesh[:-1]))
    # a = span
    # b = Area_ell/(np.pi*a/2)
    
    # m_vals = wingpropinfo.vlm_mesh
    # span = m_vals[0, :, 1] / (m_vals[0, -1, 1] - m_vals[0, 0, 1])
    # span = span - (span[0] + 0.5)
    # lift_area = np.sum(Clc_wing_coupled_opt * (span[1:] - span[:-1]))
    # span_ctr_pnts = np.array([0.5*(span[index]+span[index+1]) for index in range(len(span)-1)])
    # lift_ell = 4 * lift_area / np.pi * np.sqrt(1 - (2 * span_ctr_pnts) ** 2)
    
    twist_orig_isowing = first_case_isowing.outputs['OPENAEROSTRUCT.wing.geometry.twist'][0]
    twist_opt_isowing = last_case_isowing.outputs['OPENAEROSTRUCT.wing.geometry.twist'][0]
    twist_opt_coupled = last_case_coupled.outputs['OPENAEROSTRUCT.wing.geometry.twist'][0]
    
    x_prop = np.linspace(-0.1185, 0.1185, 100)
    y_prop = prop_circle(r=0.1185, x=x_prop)
        
    subplots_wingprop(  design_variable_array=[[isowingmesh_ctr_pnts, isowingmesh_ctr_pnts, vlmmesh_ctr_pnts],
                                               [isowingmesh, isowingmesh, vlmmesh_ctr_pnts],
                                               [isowingmesh, isowingmesh, vlm_mesh]],
                        nr_plots=3,
                        xlabel=r'Wing spanwise location, $m$', ylabel=[r'$C_l \cdot c ,m$', r'Chord, $m$', r'Twist, deg'],
                        savepath=os.path.join(savedir, 'wing_results'), 
                        prop_circle=[x_prop, y_prop],
                        noprop=True,
                        clc=[Clc_wing_orig, Clc_wing_iso_opt, Clc_wing_coupled_opt],
                        chord=[chord_orig, chord_opt_isowing, chord_opt_coupled],
                        twist=[twist_orig_isowing, twist_opt_isowing, twist_opt_coupled],
                        )
    
    veldistr_orig = first_case_isoprop.outputs['HELIX_0.om_helix.rotorcomp_0_velocity_distribution']
    veldistr_iso_opt = last_case_isoprop.outputs['HELIX_0.om_helix.rotorcomp_0_velocity_distribution']
    veldistr_coupled = last_case_coupled.outputs['HELIX_0.om_helix.rotorcomp_0_velocity_distribution']
    
    try:
        twist_orig = first_case_isoprop.outputs['HELIX_0.om_helix.geodef_parametric_0_twist']
        twist_iso_opt = last_case_isoprop.outputs['HELIX_0.om_helix.geodef_parametric_0_twist']
        twist_coupled = last_case_coupled.outputs['DESIGNVARIABLES.rotor_1_twist']
    except:
        twist_orig = first_case_isoprop.outputs['DESIGNVARIABLES.rotor_0_twist']
        twist_iso_opt = last_case_isoprop.outputs['DESIGNVARIABLES.rotor_0_twist']
        twist_coupled = last_case_coupled.outputs['DESIGNVARIABLES.rotor_0_twist']
    
    propspan = np.linspace(0, 1, len(twist_coupled))
    propspan_veldistr = np.linspace(0, 1, len(veldistr_coupled))
    subplots_prop(      design_variable_array=[[propspan_veldistr, propspan_veldistr, propspan_veldistr],
                                               [propspan, propspan, propspan]], 
                        nr_plots=2,
                        xlabel='Normalised propeller blade location, r/R',
                        ylabel=['Velocity, m/s', 'Twist, deg'],
                        savepath=os.path.join(savedir, 'prop_results'), 
                        vel_distr=[veldistr_orig, veldistr_iso_opt, veldistr_coupled],
                        twist=[twist_orig, twist_iso_opt, twist_coupled]
                        )

def subplots_wingprop(  design_variable_array: np.array, nr_plots: int,
                        xlabel: str, ylabel: str,
                        savepath: str, 
                        prop_circle: list,
                        noprop: bool,
                        **kwargs)->None:
    
    colors = get_niceColors()
    delftcolors = get_delftColors()
    supernicecolors = get_SuperNiceColors()
    margin = 1.03
    linewidth = 1.
    fontsize=12
    y_size = 9
    
    plt.style.use(niceplots.get_style())
    # plt.rc('font', size=16)
    _, ax = plt.subplots(nr_plots, figsize=(6, 6), sharex=True)
        
    spanwise = design_variable_array

    for iplot, key in enumerate(kwargs.keys()):
        original  = kwargs[key][0]
        optimised_isowing  = kwargs[key][1]
        optimised_coupled  = kwargs[key][2]
        ymax = np.max([max(original), np.max(optimised_isowing), np.max(optimised_coupled)])*margin
        
        ax[iplot].plot(spanwise[iplot][0], original,
                label=f'Initial, isolated', color='Orange', linestyle='dashed', linewidth=linewidth)
        ax[iplot].plot(spanwise[iplot][1], optimised_isowing,
                label=f'Optimized, isolated', color='b', linewidth=linewidth)
        ax[iplot].plot(spanwise[iplot][2], optimised_coupled,
                label=f'Optimized, coupled', linewidth=linewidth)
        
        # Plot elliptical lift curve if given
        # if 'clc' in key:
        #     ax[iplot].plot(spanwise[iplot], kwargs[key][2],
        #         label='Elliptical lift curve', color='black', linewidth=linewidth/2)
        
        if not noprop:
            x_prop = prop_circle[0]
            y_prop = prop_circle[1]*ymax/(max(spanwise[iplot][0])-min(spanwise[iplot][0]))*8/(1.5*y_size/nr_plots)-0.01
            
            for prop_loc in [-0.332, 0.332]:
                if prop_loc<0:
                    x_prop_plot = x_prop+prop_loc
                    ax[iplot].plot(x_prop_plot, y_prop, color='grey', linestyle='dashed', label='Propeller', linewidth=0.5)
                else:
                    x_prop_plot = x_prop+prop_loc
                    ax[iplot].plot(x_prop_plot, y_prop, color='grey', linestyle='dashed', linewidth=0.5)
            
        # for prop_loc in [-0.332, 0.332]:
        #     x_prop_plot = x_prop+prop_loc
        #     ax[iplot].plot(np.ones(10)*(0.332-0.1185), np.linspace(0, 0.2, 10), color='black', label='Propeller', linewidth=0.5)
        
        if iplot==2:
            ax[iplot].set_xlabel(xlabel, fontweight='ultralight')
        ax[iplot].set_ylabel(ylabel[iplot], fontweight='ultralight')

        ax[iplot].set_ylim((
            -0.01,
            ymax)
        )
        ax[iplot].set_xlim((
            np.min(spanwise[iplot][0])*margin,
            np.max(spanwise[iplot][0])*margin)
        )
        if iplot==0:
            ax[iplot].legend()
        niceplots.adjust_spines(ax[iplot], outward=True)

    plt.savefig(savepath)
    
    plt.clf()
    plt.close()

def subplots_prop(      design_variable_array: np.array, nr_plots: int,
                        xlabel: str, ylabel: str,
                        savepath: str, 
                        **kwargs)->None:
    
    colors = get_niceColors()
    delftcolors = get_delftColors()
    supernicecolors = get_SuperNiceColors()
    margin = 1.03
    linewidth = 1.
    fontsize=12
    y_size = 10
    
    plt.style.use(niceplots.get_style())
    # plt.rc('font', size=16)
    _, ax = plt.subplots(nr_plots, figsize=(6, 6), sharex=True)
        
    spanwise = design_variable_array

    for iplot, key in enumerate(kwargs.keys()):
        original  = kwargs[key][0]
        optimised_isowing  = kwargs[key][1]
        optimised_coupled  = kwargs[key][2]
        ymax = np.max([max(original), np.max(optimised_isowing), np.max(optimised_coupled)])*margin
        ymin = np.min([min(original), np.min(optimised_isowing), np.min(optimised_coupled)])/margin
        
        if iplot==0:
            ax[iplot].plot(spanwise[iplot][0][16:], original[16:],
                    label=f'Initial', color='Orange', linestyle='dashed', linewidth=linewidth)
            ax[iplot].plot(spanwise[iplot][1][16:], optimised_isowing[16:],
                    label=f'Optimized, isolated', color='b', linewidth=linewidth)
            ax[iplot].plot(spanwise[iplot][2][16:], optimised_coupled[16:],
                    label=f'Optimized, coupled', linewidth=linewidth)
        else:
            start=3
            ax[iplot].plot(spanwise[iplot][0][start:], original[start:],
                    label=f'Initial', color='Orange', linestyle='dashed', linewidth=linewidth)
            ax[iplot].plot(spanwise[iplot][1][start:], optimised_isowing[start:],
                    label=f'Optimized, isolated', color='b', linewidth=linewidth)
            ax[iplot].plot(spanwise[iplot][2][start:], optimised_coupled[start:],
                    label=f'Optimized, coupled', linewidth=linewidth)

        if iplot==1:
            ax[iplot].set_xlabel(xlabel, fontweight='ultralight')
        ax[iplot].set_ylabel(ylabel[iplot], fontweight='ultralight')
        ax[iplot].set_ylim((
            ymin,
            ymax)
        )
        ax[iplot].set_xlim((
            0.2, # bcs the prowim sucks
            np.max(spanwise[iplot][0])*margin)
        )
        if iplot==0:
            ax[iplot].legend()
        niceplots.adjust_spines(ax[iplot], outward=True)

    plt.savefig(savepath)
    
    plt.clf()
    plt.close()
