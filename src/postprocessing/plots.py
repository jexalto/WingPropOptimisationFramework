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

# --- External ---
import matplotlib.pyplot as plt
import matplotlib as mpl
import niceplots
from openmdao.recorders.sqlite_reader import SqliteCaseReader
import numpy as np
import pandas as pd
from scipy.interpolate import interp1d

# TODO: write specific propeller, wing and prop-wing plotting functions, it's very messy right now

logging.getLogger('matplotlib.font_manager').disabled = True

def stackedplots_prop(db_name: str,
              wingpropinfo: WingPropInfo,
              savedir: str)->None:
    database = SqliteCaseReader(db_name, pre_load=True)

    # === Misc variables ===
    span = wingpropinfo.wing.span

    # === Objective, constraints and DVs ===
    # Original
    first_case = database.get_cases()[0]

    # Optimised
    last_case = database.get_cases()[-1]
    
    veldistr_orig = first_case.outputs['HELIX_0.om_helix.rotorcomp_0_velocity_distribution']
    veldistr_opt = last_case.outputs['HELIX_0.om_helix.rotorcomp_0_velocity_distribution']
    
    twist_orig = first_case.outputs[f'HELIX_{0}.om_helix.geodef_parametric_0_twist']
    twist_opt = last_case.outputs[f'HELIX_{0}.om_helix.geodef_parametric_0_twist']
    
    subplots_prop(  design_variable_array=[np.linspace(0, 1, len(veldistr_orig)),np.linspace(0., 1, len(twist_orig))],
                    nr_plots=2,
                    xlabel=r'Normalised blade location, $r/R$', ylabel=[r'Exit velocity, $m/s$', r'Twist, $deg$'],
                    savepath=os.path.join(savedir, 'prop_results'), 
                    veldistr=[veldistr_orig, veldistr_opt],
                    twist=[twist_orig, twist_opt])

def stackedplots_wing(db_name: str,
              wingpropinfo: WingPropInfo,
              savedir: str,
              noprop=False)->None:
    database = SqliteCaseReader(db_name, pre_load=True)

    # === Misc variables ===
    span = wingpropinfo.wing.span

    # === Objective, constraints and DVs ===
    # Original
    first_case = database.get_cases()[0]

    # Optimised
    last_case = database.get_cases()[-1]

    # === Misc variables ===
    spanwise_mesh = wingpropinfo.vlm_mesh_control_points
    vlm_mesh = wingpropinfo.vlm_mesh[0, :, 1]
    
    # === Wing plotting ===
    Cl_wing_orig = first_case.outputs['OPENAEROSTRUCT.AS_point_0.wing_perf.Cl']
    Cl_wing_opt = last_case.outputs['OPENAEROSTRUCT.AS_point_0.wing_perf.Cl']
    
    try:
        chord_orig = first_case.outputs['OPENAEROSTRUCT.wing.geometry.chord'][0]
        chord_opt = last_case.outputs['OPENAEROSTRUCT.wing.geometry.chord'][0]
        
        chord_nodes_orig = np.zeros(len(Cl_wing_opt))
        chord_nodes_orig = [(chord_orig[index]+chord_orig[index+1])/2 for index in range(len(chord_orig)-1)]
        
        chord_nodes_opt = np.zeros(len(Cl_wing_opt))
        chord_nodes_opt = [(chord_opt[index]+chord_opt[index+1])/2 for index in range(len(chord_opt)-1)]
    
    except:
        chord_nodes_orig = np.ones(len(Cl_wing_opt))
        chord_nodes_opt = np.ones(len(Cl_wing_opt))
        
    Clc_wing_orig = Cl_wing_orig*chord_nodes_orig
    Clc_wing_opt = Cl_wing_opt*chord_nodes_opt
    
    Area_ell = 2*sum(Clc_wing_opt*(vlm_mesh[1:]-vlm_mesh[:-1]))
    a = span
    b = Area_ell/(np.pi*a/2)
    
    m_vals = wingpropinfo.vlm_mesh
    span = m_vals[0, :, 1] / (m_vals[0, -1, 1] - m_vals[0, 0, 1])
    span = span - (span[0] + 0.5)
    lift_area = np.sum(Clc_wing_opt * (span[1:] - span[:-1]))
    span_ctr_pnts = np.array([0.5*(span[index]+span[index+1]) for index in range(len(span)-1)])
    lift_ell = 4 * lift_area / np.pi * np.sqrt(1 - (2 * span_ctr_pnts) ** 2)
    
    twist_orig = first_case.outputs['OPENAEROSTRUCT.wing.geometry.twist'][0]
    twist_opt = last_case.outputs['OPENAEROSTRUCT.wing.geometry.twist'][0]
    
    x_prop = np.linspace(-0.1185, 0.1185, 100)
    y_prop = prop_circle(r=0.1185, x=x_prop)
        
    subplots_wingprop(design_variable_array=[spanwise_mesh, vlm_mesh, vlm_mesh], nr_plots=3,
                        xlabel=r'Wing spanwise location, $m$', ylabel=[r'$C_l \cdot c ,m$', r'Chord, $m$', r'Twist, deg'],
                        savepath=os.path.join(savedir, 'wing_results'), 
                        prop_circle=[x_prop, y_prop],
                        noprop=True,
                        clc=[Clc_wing_orig, Clc_wing_opt, lift_ell],
                        chord=[chord_orig, chord_opt],
                        twist=[twist_orig, twist_opt],
                        )

def all_plots(db_name: str,
              wingpropinfo: WingPropInfo,
              savedir: str,
              *kwargs) -> None:
    database = SqliteCaseReader(db_name, pre_load=True)

    # === Misc variables ===
    span = wingpropinfo.wing.span

    # === Objective, constraints and DVs ===
    # Original
    first_case = database.get_cases()[0]
    design_variables_orig = first_case.get_design_vars()
    constraints_orig = first_case.get_constraints()
    objective_orig = first_case.get_objectives()

    # Optimised
    last_case = database.get_cases()[-1]
    design_variables_opt = last_case.get_design_vars()
    constraints_opt = last_case.get_constraints()
    objective_opt = last_case.get_objectives()

    # === Misc variables ===
    spanwise_mesh = wingpropinfo.vlm_mesh_control_points
    vlm_mesh = wingpropinfo.vlm_mesh[0, :, 1]
    try:
        for misckey in first_case.outputs.keys():
            if 'velocity_distribution' in misckey:
                veldistr_orig = first_case.outputs[misckey]
                veldistr_opt = last_case.outputs[misckey]

                # === Plotting misc variables ===
                var_x = np.linspace(0, 1, len(veldistr_orig))
                optimisation_result_plot(design_variable_array=var_x, original=veldistr_orig, optimised=veldistr_opt,
                                         label=r"$V$", xlabel=r'Normalised propeller blade', ylabel=r"Velocity, $m/s$",
                                         savepath=os.path.join(savedir, f'vel_distr_prop'))

            elif 'AS_point_0.wing_perf.Cl' in misckey:
                Cl_wing_orig = first_case.outputs[misckey]
                Cl_wing_opt = last_case.outputs[misckey]
                
                try:
                    chord_orig = first_case.outputs['OPENAEROSTRUCT.wing.geometry.chord'][0]
                    chord_opt = last_case.outputs['OPENAEROSTRUCT.wing.geometry.chord'][0]
                    
                    chord_nodes_orig = np.zeros(len(Cl_wing_opt))
                    chord_nodes_orig = [(chord_orig[index]+chord_orig[index+1])/2 for index in range(len(chord_orig)-1)]
                    
                    chord_nodes_opt = np.zeros(len(Cl_wing_opt))
                    chord_nodes_opt = [(chord_opt[index]+chord_opt[index+1])/2 for index in range(len(chord_opt)-1)]
                
                except:
                    chord_nodes_orig = np.ones(len(Cl_wing_opt))
                    chord_nodes_opt = np.ones(len(Cl_wing_opt))
                    
                Clc_wing_orig = Cl_wing_orig*chord_nodes_orig
                Clc_wing_opt = Cl_wing_opt*chord_nodes_opt
                
                Area_ell = 2*sum(Clc_wing_opt*(vlm_mesh[1:]-vlm_mesh[:-1]))
                a = span
                b = Area_ell/(np.pi*a/2)
                
                m_vals = wingpropinfo.vlm_mesh
                span = m_vals[0, :, 1] / (m_vals[0, -1, 1] - m_vals[0, 0, 1])
                span = span - (span[0] + 0.5)
                lift_area = np.sum(Clc_wing_opt * (span[1:] - span[:-1]))
                span_ctr_pnts = np.array([0.5*(span[index]+span[index+1]) for index in range(len(span)-1)])
                lift_ell = 4 * lift_area / np.pi * np.sqrt(1 - (2 * span_ctr_pnts) ** 2)

                # === Plotting misc variables ===
                var_x = np.linspace(-span/2, span/2, len(Cl_wing_orig))
                optimisation_result_plot(design_variable_array=spanwise_mesh,
                                         original=Clc_wing_orig,
                                         optimised=Clc_wing_opt,
                                         label=r"$C_l \cdot c$", xlabel=r'Wing spanwise location', ylabel=r"Lift coefficient, $C_l \cdot c$",
                                         savepath=os.path.join(savedir, f'Cl_Wing'),
                                         lift_ell=lift_ell)

            elif 'wing.geometry.twist' in misckey:
                twist_orig = first_case.outputs[misckey][0]
                twist_opt = last_case.outputs[misckey][0]

                # === Plotting misc variables ===
                var_x = wingpropinfo.vlm_mesh[0, :, 1]
                optimisation_result_plot(design_variable_array=var_x, original=twist_orig, optimised=twist_opt,
                                         label=r"$Twist, deg$", xlabel=r'Wing spanwise location', ylabel=r"$Twist, deg$",
                                         savepath=os.path.join(savedir, f'Wing_twist_DV'))
                
            elif misckey=='OPENAEROSTRUCT.wing.geometry.chord':
                chord_orig = first_case.outputs[misckey][0]*wingpropinfo.wing.chord[0]
                chord_opt = last_case.outputs[misckey][0]*wingpropinfo.wing.chord[0]

                # === Plotting misc variables ===
                var_x = wingpropinfo.vlm_mesh[0, :, 1]
                optimisation_result_plot(design_variable_array=var_x, original=chord_orig, optimised=chord_opt,
                                         label=r"$Chord, m$", xlabel=r'Wing spanwise location', ylabel=r"$Chord, m$",
                                         savepath=os.path.join(savedir, f'Wing_chord_DV'))

    except Exception as e:
        print(f'No CL found: {e}')

    # === Plotting design variables ===
    for index, dv_key in enumerate(design_variables_orig.keys()):
        variable_orig = design_variables_orig[dv_key]
        variable_opt = design_variables_opt[dv_key]

        if len(variable_orig) != 1:
            str_lst = dv_key.split(".")
            var_name = str_lst[-1]

            if 'rotor' in var_name or 'geodef_parametric' in var_name:
                # Propeller Plotting
                variable = var_name[8:]
                variable = variable.split('_')[-1]
                variable = variable.capitalize()
                variable_label = r'Twist, $deg$'
                var_x = np.linspace(0, 1, len(variable_orig))
                optimisation_result_plot(design_variable_array=var_x, original=variable_orig, optimised=variable_opt,
                                         label=variable, xlabel=r'Propeller spanwise location $y$', ylabel=variable_label,
                                         savepath=os.path.join(savedir, f'Prop_{variable}_{index}'.lower()))

            else:
                # Wing Plotting
                var_x = np.linspace(-span/2, span/2, len(variable_orig))
                optimisation_result_plot(design_variable_array=var_x,
                                         original=variable_orig,
                                         optimised=variable_opt,
                                         label=var_name, xlabel=r'Wing spanwise location', ylabel=var_name,
                                         savepath=os.path.join(savedir, f'Wing_{var_name}'.lower()))

    # === Plotting objectives ===
    for dv_key in objective_orig.keys():
        variable_orig = objective_orig[dv_key]
        variable_opt = objective_opt[dv_key]

        if len(variable_orig) != 1:
            str_lst = dv_key.split(".")
            var_name = str_lst[-1]

            if var_name[:5] == 'rotor':
                # Propeller Plotting
                variable = var_name[8:]
                var_x = np.linspace(0, 1, len(variable_orig))
                optimisation_result_plot(design_variable_array=var_x, original=variable_orig, optimised=variable_opt,
                                         label=var_name, xlabel=r'Propeller spanwise location $y$', ylabel=var_name,
                                         savepath=os.path.join(savedir, f'Prop_{variable}'))

            else:
                # Wing Plotting
                var_x = np.linspace(-span/2, span/2, len(variable_orig))
                optimisation_result_plot(design_variable_array=var_x,
                                         original=variable_orig,
                                         optimised=variable_opt,
                                         label=var_name, xlabel=r'Wing spanwise location', ylabel=var_name,
                                         savepath=os.path.join(savedir, f'Wing_{var_name}'))

    # === Plotting constraints ===
    for dv_key in constraints_opt.keys():
        variable_orig = constraints_opt[dv_key]
        variable_opt = constraints_orig[dv_key]

        if len(variable_orig) != 1:
            str_lst = dv_key.split(".")
            var_name = str_lst[-1]

            if var_name[:5] == 'rotor':
                # Propeller Plotting
                variable = var_name[8:]
                var_x = np.linspace(0, 1, len(variable_orig))
                optimisation_result_plot(design_variable_array=var_x, original=variable_orig, optimised=variable_opt,
                                         label=var_name, xlabel=r'Propeller spanwise location $y$', ylabel=var_name,
                                         savepath=os.path.join(savedir, f'Prop_{variable}'))

            else:
                # Wing Plotting
                var_x = np.linspace(-span/2, span/2, len(variable_orig))
                optimisation_result_plot(design_variable_array=var_x, original=variable_orig, optimised=variable_opt,
                                         label=var_name, xlabel=r'Wing spanwise location [$m$]', ylabel=rf'${var_name}$',
                                         savepath=os.path.join(savedir, f'Wing_{var_name}'))

    scatter_plots(db_name=db_name,
                  savedir=savedir)


def plot_optimality(SNOPT_output: str):
    ...


def scatter_plots(db_name: str,
                  savedir: str):
    database = SqliteCaseReader(db_name, pre_load=True)

    # === Objective, constraints and DVs ===
    # Original
    cases = database.get_cases()
    for varkey in cases[0].outputs.keys():
        var = []
        if np.size(cases[0][varkey]) == 1:
            for case in cases:
                var.append(case.outputs[varkey])

            design_variable_array = np.linspace(0, len(var), len(var))

            ylabel = varkey.split('.')[-1]

            optimisation_singlevalue_results(design_variable_array=design_variable_array,
                                             xlabel='Iterations', ylabel=ylabel,
                                             savepath=os.path.join(
                                                 savedir, ylabel),
                                             variable=var)


def subplots_prop(design_variable_array: np.array, nr_plots: int,
             xlabel: str, ylabel: str,
             savepath: str, 
             **kwargs)->None:
    
    colors = get_niceColors()
    delftcolors = get_delftColors()
    supernicecolors = get_SuperNiceColors()
    margin = 1.03
    linewidth = 1.
    fontsize=12
    y_size = 9
    
    plt.style.use(niceplots.get_style())
    plt.rc('font', size=16)
    _, ax = plt.subplots(nr_plots, figsize=(6, 6), sharex=True)
        
    spanwise = design_variable_array

    for iplot, key in enumerate(kwargs.keys()):
        original  = kwargs[key][0]
        optimised  = kwargs[key][1]
        ymax = np.max([np.max(original), np.max(optimised)])*margin
        ymin = np.min([np.min(original), np.min(optimised)])*1/margin
        
        if iplot==0:
            start = 16
            ax[iplot].plot(spanwise[iplot][start:], original[start:],
                    label=f'Initial', color='Orange', linewidth=linewidth)
            ax[iplot].plot(spanwise[iplot][start:], optimised[start:],
                    label=f'Optimised', color='b', linestyle='dashed', linewidth=linewidth)
        else:
            start = 4
            ax[iplot].plot(spanwise[iplot][start:], original[start:],
                    label=f'Initial', color='Orange', linewidth=linewidth)
            ax[iplot].plot(spanwise[iplot][start:], optimised[start:],
                    label=f'Optimised', color='b', linestyle='dashed', linewidth=linewidth)

        if iplot==1:
            ax[iplot].set_xlabel(xlabel, fontweight='ultralight')
        ax[iplot].set_ylabel(ylabel[iplot], fontweight='ultralight')

        ax[iplot].set_ylim((
            ymin,
            ymax)
        )
        ax[iplot].set_xlim((
            0.2,
            np.max(spanwise[iplot])*margin)
        )
        if iplot==0:
            ax[iplot].legend()
        niceplots.adjust_spines(ax[iplot], outward=True)

    plt.savefig(savepath)
    
    plt.clf()
    plt.close()

def subplots_wingprop(design_variable_array: np.array, nr_plots: int,
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
    # plt.rc('font', size=14)
    _, ax = plt.subplots(nr_plots, figsize=(6, 6), sharex=True)
        
    spanwise = design_variable_array

    for iplot, key in enumerate(kwargs.keys()):
        original  = kwargs[key][0]
        optimised  = kwargs[key][1]
        ymax = np.max([max(original), np.max(optimised)])*margin
        
        ax[iplot].plot(spanwise[iplot], original,
                label=f'Initial', color='Orange', linewidth=linewidth)
        ax[iplot].plot(spanwise[iplot], optimised,
                label=f'Optimised', color='b', linestyle='dashed', linewidth=linewidth)
        
        # Plot elliptical lift curve if given
        if 'clc' in key:
            ax[iplot].plot(spanwise[iplot], kwargs[key][2],
                label='Elliptical lift curve', color='black', linewidth=linewidth/2)
        
        if not noprop:
            x_prop = prop_circle[0]
            y_prop = prop_circle[1]*ymax/(max(spanwise[iplot])-min(spanwise[iplot]))*8/(1.5*y_size/nr_plots)-0.025
            
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
            -0.025,
            ymax)
        )
        ax[iplot].set_xlim((
            np.min(spanwise[iplot])*margin,
            np.max(spanwise[iplot])*margin)
        )
        if iplot==0:
            ax[iplot].legend()
        niceplots.adjust_spines(ax[iplot], outward=True)

    plt.savefig(savepath)
    
    plt.clf()
    plt.close()

def optimisation_result_plot(design_variable_array: np.array, original: np.array, optimised: np.array,
                             label: str, xlabel: str, ylabel: str,
                             savepath: str, **kwargs) -> None:
    colors = get_niceColors()
    delftcolors = get_delftColors()
    supernicecolors = get_SuperNiceColors()
    margin = 1.015
    ymax = np.max([max(original), np.max(optimised)])

    plt.style.use(niceplots.get_style())
    _, ax = plt.subplots(figsize=(8, 6))

    spanwise = design_variable_array
    ax.plot(spanwise, original,
            label=f'{label}, original', color='Orange')
    ax.plot(spanwise, optimised,
            label=f'{label}, optimised', color='b', linestyle='dashed')
    
    for plot in kwargs.values():
        ax.plot(spanwise, plot,
            label='Elliptical lift curve', color='black', linestyle='dashed', linewidth=1.)

    # ax.plot([0.332-0.12, 0.332-0.12], [0., 0.25], color=delftcolors['Grey'], linewidth=0.5)
    # ax.plot([0.332+0.12, 0.332+0.12], [0., 0.25], color=delftcolors['Grey'], linewidth=0.5)
    # ax.arrow(x=0.332, y=0.02, dx=0.06, dy=0.)

    # ax.text(x=0.332-0.1, y=0.03, s='Propeller', fontdict={'fontsize': mpl.rcParams['axes.titlesize'],
    #                                                         'fontweight': 'ultralight',
    #                                                         'fontsize': 12})

    ax.set_xlabel(xlabel, fontweight='ultralight')
    ax.set_ylabel(ylabel, fontweight='ultralight')

    ax.set_ylim((
        0,
        np.max([max(original), np.max(optimised)])*margin)
    )
    ax.set_xlim((
        np.min(spanwise)*margin,
        np.max(spanwise)*margin)
    )
    ax.legend()
    niceplots.adjust_spines(ax, outward=True)

    plt.savefig(savepath)
    
    plt.clf()
    plt.close()

def optimisation_singlevalue_results(design_variable_array: np.array,
                                     xlabel: str, ylabel: str,
                                     savepath: str,
                                     **kwargs) -> None:
    plt.style.use(niceplots.get_style())
    _, ax = plt.subplots(figsize=(10, 7))

    spanwise = design_variable_array
    for key in kwargs.keys():
        ax.plot(spanwise, kwargs[key])

    ax.set_xlabel(xlabel, fontweight='ultralight')
    ax.set_ylabel(ylabel, fontweight='ultralight')
    niceplots.adjust_spines(ax, outward=True)

    plt.savefig(savepath)
    
    plt.clf()
    plt.close()


# if __name__ == '__main__':
    # scatter_plots(savedir='.',
    #     db_name='/home/mdolabuser/mount/code/framework/WingPropOptimisationFramework/examples/optimisation/results/data.db')
#     all_plots(db_name='/home/mdolabuser/mount/code/framework/WingPropOptimisationFramework/examples/optimisation/results/data.db',
#               wingpropinfo=PROWIM_wingpropinfo,
#               savedir='.')
