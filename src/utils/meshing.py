# --- Built-ins ---
from math import ceil

# --- Internal ---

# --- External ---
import numpy as np


def meshing(span: float, chord: float, prop_locations: np.array, prop_radii: np.array, nr_props: int, 
            spanwise_discretisation_wing: int, spanwise_panels_propeller: int):
    wingtipprop = False
    y_vlm = np.array([], order='F')
    spanwise_nodes_propeller = spanwise_panels_propeller+1

    nr_wing_regions = nr_props+1
    
    # check for wingtip propellers
    if prop_locations[0]-max(prop_radii[0])<=-span/2:
        nr_wing_regions -= 2
        wingtipprop = True
    
    wing_panels_regional = int(spanwise_discretisation_wing/nr_wing_regions)
    
    # Check whether ny is odd
    ny = wing_panels_regional*(nr_props+1)+spanwise_nodes_propeller*nr_props

    if ny%2==0:
        wing_panels_regional+=1
    
    # Update ny
    ny = wing_panels_regional*(nr_props+1)+spanwise_nodes_propeller*nr_props    
    
    y_vlm_prop = np.zeros((nr_props, spanwise_nodes_propeller))
    
    # === Wingtip meshing ===
    if wingtipprop: # the no wingtip propeller procedure
        for iprop in range(nr_props): # determine propeller mesh
            iprop_loc, iprop_radius = prop_locations[iprop], prop_radii[iprop]
            iprop_loc = prop_locations[iprop]
            prop_left = iprop_loc-max(iprop_radius)
            prop_right = iprop_loc+max(iprop_radius)
            
            y_vlm_prop[iprop, :] = np.linspace(prop_left, prop_right, spanwise_nodes_propeller)
        
        wing_distance_total = 0.
        wing_panels_regional_lst = []
        for iwing in range(nr_wing_regions):# determine meshing, so that all panels are almost the same size
            wing_distance_total += abs(y_vlm_prop[iwing, -1]-y_vlm_prop[iwing+1, 0])
        
        for iwing in range(nr_wing_regions):# determine meshing, so that all panels are almost the same size
            wing_region_distance = abs(y_vlm_prop[iwing, -1]-y_vlm_prop[iwing+1, 0])
            wing_panels_regional_lst.append(int(ceil(wing_region_distance/wing_distance_total*spanwise_discretisation_wing)))
        
        if sum(wing_panels_regional_lst)%2==0:
            wing_panels_regional_lst[int(nr_props/2)-1] += 1
        
        y_vlm_wing = []
        for iwing in range(nr_wing_regions): # determine wing mesh    
            y_vlm_wing.append(np.linspace(y_vlm_prop[iwing, -1], y_vlm_prop[iwing+1, 0], wing_panels_regional_lst[iwing]))

        mesh = np.array([])
        for iwing in range(nr_wing_regions): # merge wing and propeller mesh
            mesh = np.append(mesh, y_vlm_prop[iwing][:-1])
            mesh = np.append(mesh, y_vlm_wing[iwing][:-1])
        mesh = np.append(mesh, y_vlm_prop[-1, :])
        
        mesh = mesh[np.where(mesh>=-span/2)[0]]
        mesh = mesh[np.where(mesh<=span/2)[0]]
        
        # check for full span
        if mesh[0]!=-span/2:
            mesh_2D = np.append([-span/2], mesh)
        if mesh[-1]!=span/2:
            mesh_2D = np.append(mesh_2D, [span/2])
    
    
    # === No Wingtip meshing ===
    if not wingtipprop: # the no wingtip propeller procedure
        y_vlm = np.array([-span/2])
        for iprop in range(nr_props):
            start = y_vlm[-1]
            iprop_loc = prop_locations[iprop]
            prop_left = iprop_loc-max(prop_radii[iprop])
            prop_right = iprop_loc+max(prop_radii[iprop])

            new_mesh = np.linspace(start, prop_left, wing_panels_regional)
            new_mesh = np.concatenate((new_mesh[1:-1],
                                    np.linspace(prop_left, prop_right, spanwise_nodes_propeller))
                                    )
            
            y_vlm = np.concatenate((y_vlm, new_mesh))

        new_mesh = np.linspace(prop_right, span/2, wing_panels_regional)
        mesh_2D = np.concatenate((y_vlm, new_mesh[1:]))
    
    nx = 2  # number of chordwise nodal points
    ny = len(mesh_2D)
    assert (ny%2!=0)
    # number of spanwise nodal points for the outboard segment

    mesh = np.zeros((nx, ny, 3), order='F')

    mesh[:, :, 2] = 0.0
    mesh[:, :, 1] = mesh_2D
    mesh[:, :, 0] = np.zeros(ny)
    mesh[0, :, 0] = np.zeros(ny)
    mesh[1, :, 0] = np.ones(ny)*chord

    return np.array(mesh, order='F')
