# --- Built-ins ---

# --- Internal ---
from src.utils.meshing import meshing

# --- External ---
import numpy as np
import matplotlib.pyplot as plt


if __name__ == '__main__':
    # mesh = meshing(span=10, chord=0.25, prop_locations=[-5, 5], prop_radii=[[0.5, 1.], [0.5, 1.]], nr_props=2,
    #                spanwise_discretisation_wing=20, spanwise_panels_propeller=5)

    # print(mesh[0, :, 1])

    mesh = meshing(span=10, chord=0.25, prop_locations=[-5, -2.5, 2.5, 5], prop_radii=[[0.5, 1.], [0.5, 1.], [0.5, 1.], [0.5, 1.]], nr_props=4,
                   spanwise_discretisation_wing=20, spanwise_panels_propeller=5)

    print(mesh[0, :, 1])
    
    y = np.ones(len(mesh[0, :, 1]))
    plt.plot([-5, -4], [1.2, 1.2], color='k')
    plt.plot([-3.5, -1.5], [1.2, 1.2], color='k')
    plt.plot([1.5, 3.5], [1.2, 1.2], color='k')
    plt.plot([4, 5], [1.2, 1.2], color='k')
    plt.scatter(mesh[0, :, 1], y, marker='x')
    
    plt.ylim((.5, 1.5))
    plt.savefig('mesh.png')