# System imports and path initializations.
import sys
from os.path import expanduser
sys.path.insert(0, expanduser('~')+'/prog/mpc')
sys.path.insert(0, expanduser('~')+'/prog/four')
sys.path.insert(0, expanduser('~')+'/prog/geom')

# Standard imports.
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# MPC class imports.
import MPC.Plant as plant
import MPC.Optimizer as opt
import GEOM.Vehicle2D as vhc
import FOUR.Transforms as four

# Hyper parameter(s)
A = 100
Nsteps = 10
dt = 0.025
P = 12
k = 2
R = 0.25
Nx = 4
Nu = 2

file = expanduser('~')+'/prog/four/abby_pkg/sketchdata.csv'
data = pd.read_csv( file )
xTrain = (1/20)*data[ data['z']=='d' ].to_numpy()[:,:2].T
nTrain = xTrain.shape[1]
tTrain = np.array( [[i for i in range( nTrain )]] )
fvar = four.RealFourier( tTrain, xTrain )
fvar.ls( N=50 )

# Model declaration.
def model(x, u):
    dx = np.cos(x[2])*(u[0] + u[1])
    dy = np.sin(x[2])*(u[0] + u[1])
    dth = 1/R*(u[0] - u[1])
    xn = np.array( [
        x[0] + dt*dx,
        x[1] + dt*dy,
        x[2] + dt*dth,
        x[3] + dt#*np.sqrt( dx**2 + dy**2 )
    ] )
    return xn

def cost(xList, uList):
    x0 = xList[:,0,None]
    tList = np.array( [[x0[3][0] + A*i*dt for i in range( P )]] )
    pList = fvar.solve( tList )

    C = [0]
    for x, p in zip( xList.T, pList.T ):
        C = C + (x[0] - p[0])**2 + (x[1] - p[1])**2

    return C

# Main execution loop.
if __name__ == "__main__":
    # Initial position of Roomba.
    x0 = np.vstack( (fvar.solve( np.array( [[0]] ) ), [np.pi/2+0.1], [0]) )
    # x0 = np.array( [[-1],path([-1]),[0]] )

    # Initialize MPC variables.
    m_var = plant.Model( model, dt=dt )
    mpc_var = opt.ModelPredictiveControl( model, cost,
        P=P, k=k, Nx=Nx, Nu=Nu, dt=dt,
        cost_type='horizon' )
    mpc_var.setStepSize( 1.00 )

    uinit = np.zeros( (Nu,P) )
    mpc_var.setMaxIter( 1000 )
    uList = mpc_var.solve( x0, uinit, verbose=1 )
    mpc_var.setMaxIter( 10 )

    # Simulation series.
    T = 10;  Nt = round( T/dt ) + 1
    tList = np.array( [[i for i in range( Nt )]] )
    pList = fvar.solve( tList )
    # xpred = mpc_var.statePrediction( x0, uinit )

    # Vehicle variable and static initializations.
    fig, axs = plt.subplots()
    axs.plot( pList[0], pList[1],
        color='r', linestyle='--', marker='x',
        markersize=2.5, label='Desired Path',
        zorder=10 )
    v_var = vhc.Vehicle2D( x0[:2], radius=R,
        fig=fig, axs=axs, tail_length=1000, zorder=25 )

    # Initialize forward tail and plot.
    xpred = mpc_var.statePrediction( x0, uList )[:2,:]
    v_var.initForwardTail( xpred, zorder=25 )
    v_var.draw()

    # plt.axis( [-6, 6, -6, 6] )
    plt.gca().set_aspect( 'equal', adjustable='box' )
    plt.show( block=0 )

    # Simulation loop.
    x = x0
    u = uList
    input( "\nPress ENTER to start simulation loop..." )
    for i in range( Nt ):
        # Calculate optimal controls.
        u = mpc_var.solve( x, u, verbose=1 )

        # Plot forward tail.
        xpred = mpc_var.statePrediction( x, u )[:2,:]
        v_var.updateForwardTail( xpred )

        # Update state and animation.
        for _ in range( Nsteps ):
            x = m_var.prop( x, u[:,0,None] )
        v_var.update( x[:2] )
        plt.pause( 1e-3 )

    input( "\nPress ENTER to close program..." )

    # # Test path generator.
    # axs.plot( xList[0], xList[1],
    #     color='yellowgreen', marker='+' )
    # plt.show()
