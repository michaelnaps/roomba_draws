# System imports and path initializations.
import sys
from os.path import expanduser
sys.path.insert(0, expanduser('~')+'/prog/mpc');

# Standard imports.
import numpy as np
import matplotlib.pyplot as plt

# MPC class imports.
import Helpers.Plant as plant
import Helpers.Optimizer as opt
import Helpers.Vehicle2D as vhc

# Hyper parameter(s)
Nx = 3;
Nu = 2;
P = 100;
R = 1;


# Model declaration.
def model(x, u):
    xn = np.array( [
        x[0] + dt*math.cos(x[2])*(u[0] + u[1]),
        x[1] + dt*math.sin(x[2])*(u[0] + u[1]),
        x[2] + dt*1/R*(u[0] - u[1])
    ] );
    return xn;

def cost(x, u):
    pass;

# Generate path list.
def path(xcoord):
    ycoord = 0.25*np.sin( xcoord ) + 0.4*np.sin( xcoord )**2 + np.sin( xcoord )**3 + 0.1*np.sin( xcoord )**4;
    return ycoord;

def pathSteps(xstate):
    xcoord = xstate[0];
    pList = np.empty( (2, P) );
    for i in range( P ):
        ycoord = path( xcoord );
        pList[:,i] = np.hstack( (xcoord, ycoord) );
        xcoord = xcoord + 0.1;
    return pList;

# Main execution loop.
if __name__ == "__main__":
    # Initial position of Roomba.
    x0 = np.array( [[0],[0],[0]] );

    # Initialize MPC variables.
    mpc_var = opt.ModelPredictiveControl( model, cost );

    # Test path generator.
    pList = pathSteps( xtest );
    fig, axs = plt.subplots();
    axs.plot( pList[0], pList[1],
        color='r', linestyle='--',
        marker='x', markersize=5.0,
        label='Desired Path' );
    plt.show();
