# System imports and path initializations.
import sys
from os.path import expanduser
sys.path.insert(0, expanduser('~')+'/prog/mpc');

# Standard imports.
import numpy as np
import matplotlib.pyplot as plt

# MPC class imports.
import MPC.Plant as plant
import MPC.Optimizer as opt
import MPC.Vehicle2D as vhc

# Hyper parameter(s)
dt = 0.02;
P = 12;
k = 2;
R = 0.25;
Nx = 3;
Nu = 2;


# Model declaration.
def model(x, u):
    xn = np.array( [
        x[0] + dt*np.cos(x[2])*(u[0] + u[1]),
        x[1] + dt*np.sin(x[2])*(u[0] + u[1]),
        x[2] + dt*1/R*(u[0] - u[1])
    ] );
    return xn;

def cost(xList, uList):
    x0 = xList[:,0,None];
    pList = pathSteps( x0, beta=k*dt );

    C = [0];
    for x, p in zip( xList.T, pList.T ):
        C = C + (x[0] - p[0])**2 + (x[1] - p[1])**2;

    return C;

# Generate path list.
def path(xcoord):
    # ycoord = np.cos( xcoord );
    ycoord = 0.25*np.cos( xcoord ) \
        + 0.4*np.cos( xcoord )**2 \
        + np.cos( xcoord )**3 \
        + 0.1*np.cos( xcoord )**4;
    return ycoord;

def pathSteps(xstate, N=P+1, beta=dt):
    xcoord = xstate[0];
    pList = np.empty( (2, N) );
    for i in range( N ):
        ycoord = path( xcoord );
        pList[:,i] = np.hstack( (xcoord, ycoord) );
        xcoord = xcoord + beta;
    return pList;

# Main execution loop.
if __name__ == "__main__":
    # Initial position of Roomba.
    x0 = np.array( [[0],path([0]),[0]] );

    # Initialize MPC variables.
    m_var = plant.Model( model, dt=dt );
    mpc_var = opt.ModelPredictiveControl( model, cost,
        P=P, k=k, Nx=Nx, Nu=Nu, dt=dt,
        cost_type='horizon' );
    mpc_var.setStepSize( 1.00 );

    uinit = np.zeros( (Nu,P) );
    mpc_var.setMaxIter( 1000 );
    uList = mpc_var.solve( x0, uinit, verbose=1 );
    mpc_var.setMaxIter( 10 );

    # Simulation series.
    T = 10;  Nt = round( T/dt ) + 1;
    tList = np.array( [[i for i in range( Nt )]] );
    pList = pathSteps( x0, N=Nt );
    # xpred = mpc_var.statePrediction( x0, uinit )

    # Vehicle variable and static initializations.
    fig, axs = plt.subplots();
    axs.plot( pList[0], pList[1],
        color='r', linestyle='--', marker='x',
        markersize=2.5, label='Desired Path',
        zorder=10 );
    v_var = vhc.Vehicle2D( model, x0[:2],
        radius=R, fig=fig, axs=axs, tail_length=1000, zorder=25 );

    # Initialize forward tail and plot.
    xpred = mpc_var.statePrediction( x0, uList )[:2,:];
    v_var.initForwardTail( xpred )
    v_var.draw();

    # Simulation loop.
    x = x0;
    u = uList;
    input( "\nPress ENTER to start simulation loop..." );
    for i in range( Nt ):
        # Calculate optimal controls.
        u = mpc_var.solve( x, u, verbose=1 );

        # Plot forward tail.
        xpred = mpc_var.statePrediction( x, u )[:2,:];
        v_var.updateForwardTail( xpred );

        x = m_var.prop( x, u[:,0,None] );
        v_var.update( x[:2] );

        # Break if sim exceeds boundaries of T.
        if x[0] > T:
            break;
    input( "\nPress ENTER to close program..." );

    # # Test path generator.
    # axs.plot( xList[0], xList[1],
    #     color='yellowgreen', marker='+' );
    # plt.show();
