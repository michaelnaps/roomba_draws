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
dt = 0.02
P = 12
k = 2
R = 0.25
L = R + 0.15  # marker offset
Nx = 4
Nu = 2

W1 = 1/20
W2 = (1/6)*W1

# Read file data.
file = expanduser('~')+'/prog/four/abby_pkg/sketchdata.csv'
data = pd.read_csv( file )
xTrain = W1*data[ data['z']=='d' ].to_numpy()[:,:2].T

# Generate time-series data.
nTrain = xTrain.shape[1]
tTrain = W2*np.array( [[i for i in range( nTrain )]] )

# Solve Fourier Series.
fvar = four.RealFourier( tTrain, xTrain )
fvar.dmd( N=50 )

# # Plot series.
# fig, axs = plt.subplots()
# axs.plot( tTrain.T, xTrain.T )
# plt.show()

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

def markerPosition(xList):
    N = xList.shape[1]
    mList = np.empty( (2, N) )
    for i, x in enumerate( xList.T ):
        mList[:,i] = np.array( [
            x[0] - L*np.cos( x[2] ),
            x[1] - L*np.sin( x[2] )
        ] )
    return mList

def cost(xList, uList):
    x0 = xList[:,0,None]
    tList = np.array( [[x0[3][0] + dt*i for i in range( P+1 )]] )
    pList = fvar.solve( tList )

    C = [0]
    for x, p in zip( xList.T, pList.T ):
        C = C + (x[0] - p[0])**2 + (x[1] - p[1])**2

    return C

# Main execution loop.
if __name__ == "__main__":
    # Initial position of Roomba.
    t0 = np.array( [[0]] )
    t1 = t0 + dt
    xy0 = fvar.solve( t0 )
    xy1 = fvar.solve( t1 )
    th0 = np.arctan( (xy1[1] - xy0[1])/(xy1[0] - xy0[0]) ) + np.pi
    x0 = np.vstack( (xy0, th0, t0) )
    print( x0 )
    # exit()

    # Initialize MPC variables.
    m_var = plant.Model( model, dt=dt )
    mpc_var = opt.ModelPredictiveControl( model, cost,
        P=P, k=k, Nx=Nx, Nu=Nu, dt=dt,
        cost_type='horizon' )
    mpc_var.setStepSize( 0.01 )

    # Example list and first input set.
    pList = fvar.solve( tTrain[:,::5] )
    uinit = np.zeros( (Nu,P) )
    mpc_var.setMaxIter( 2500 )
    uList = mpc_var.solve( x0, uinit, verbose=1 )
    mpc_var.setMaxIter( 50 )

    # Vehicle variable and static initializations.
    fig, axs = plt.subplots()
    axs.plot( pList[0,None], pList[1,None],
        color='r', linestyle='--', marker='x',
        markersize=2.5, label='Desired Path',
        zorder=10 )
    v_var = vhc.Vehicle2D( x0[:2], radius=R,
        fig=fig, axs=axs, tail_length=2500, zorder=25 )
    marker = vhc.Vehicle2D( markerPosition( x0 ), radius=R/5, color='k',
        fig=fig, axs=axs, tail_length=2500, zorder=25 )

    # Initialize forward tail and plot.
    xpred = mpc_var.statePrediction( x0, uList )[:2,:]
    v_var.initForwardTail( xpred, zorder=25 )
    marker.tail.setLineWidth( 1.5 )
    v_var.draw()
    marker.draw()

    # Format and show plot.
    plt.gca().set_aspect( 'equal', adjustable='box' )
    plt.show( block=0 )

    # Simulation variables.
    T = 20;  Nt = round( T/dt ) + 1

    # Simulation loop.
    x = x0
    u = uList
    ans = input( "\nPress ENTER to start simulation loop... " )
    if ans == 'n':  exit()
    for _ in range( Nt ):
        # Calculate optimal controls.
        u = mpc_var.solve( x, u, verbose=0 )

        # Plot forward tail.
        xpred = mpc_var.statePrediction( x, u )[:2,:]
        v_var.updateForwardTail( xpred )

        # Update state and animation.
        x = m_var.prop( x, u[:,0,None] )
        v_var.update( x[:2] )
        marker.update( markerPosition( x ) )
        plt.pause( 1e-3 )
    input( "\nPress ENTER to close program..." )
