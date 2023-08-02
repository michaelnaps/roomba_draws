# System imports and path initializations.
import sys
from os.path import expanduser
sys.path.insert(0, expanduser('~')+'/prog/mpc')
sys.path.insert(0, expanduser('~')+'/prog/geom')

# Standard imports.
import numpy as np
import matplotlib.pyplot as plt

# MPC class imports.
import GEOM.Vehicle2D as vhc
import MPC.Plant as plant
import MPC.Optimizer as opt

# Hyper parameter(s)
A = 1
dt = A*0.02
P = 12
k = 2
R = A*0.25
L = R + A*0.10  # marker offset
Nx = 3
Nu = 2


# Model declaration.
def model(x, u):
    xn = np.array( [
        x[0] + dt*np.cos(x[2])*(u[0] + u[1]),
        x[1] + dt*np.sin(x[2])*(u[0] + u[1]),
        x[2] + dt*1/R*(u[0] - u[1])
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
    pList = pathSteps( xList[:,0,None], beta=k*dt )

    C = [0]
    for x, p in zip( xList.T, pList.T ):
        C = C + (x[0] - p[0])**2 + (x[1] - p[1])**2

    return C

# Generate path list.
def path(xcoord):
    xcoord = np.array( xcoord )/A
    ycoord = 0.25*np.cos( xcoord ) \
        + 0.4*np.cos( xcoord )**2 \
        + np.cos( xcoord )**3 \
        + 0.1*np.cos( xcoord )**4
    return A*ycoord

def pathSteps(xstate, N=P+1, beta=dt):
    xcoord = xstate[0]
    pList = np.empty( (2, N) )
    for i in range( N ):
        ycoord = path( xcoord )
        pList[:,i] = np.hstack( (xcoord, ycoord) )
        xcoord = xcoord + beta
    return pList

# Main execution loop.
if __name__ == "__main__":
    # Initial position of Roomba.
    x0 = np.array( [[0],path([0]),[0]] )
    # x0 = np.array( [[-1],path([-1]),[0]] )

    # Initialize MPC variables.
    m_var = plant.Model( model, dt=dt )
    mpc_var = opt.ModelPredictiveControl( model, cost,
        P=P, k=k, Nx=Nx, Nu=Nu, dt=dt,
        cost_type='horizon' )
    mpc_var.setStepSize( 1.00/A )

    uinit = np.zeros( (Nu,P) )
    mpc_var.setMaxIter( 1000 )
    uList = mpc_var.solve( x0, uinit, verbose=1 )
    mpc_var.setMaxIter( 10 )

    # Simulation series.
    T = A*10;  Nt = round( T/dt ) + 1
    tList = np.array( [[i for i in range( Nt )]] )
    pList = pathSteps( x0, N=Nt )
    # xpred = mpc_var.statePrediction( x0, uinit )

    # Vehicle variable and static initializations.
    fig, axs = plt.subplots()
    axs.plot( pList[0], pList[1],
        color='r', linestyle='--', marker='x',
        markersize=2.5, label='Desired Path',
        zorder=10 )
    v_var = vhc.Vehicle2D( x0[:2], radius=R,
        fig=fig, axs=axs, tail_length=250, zorder=20 )
    marker = vhc.Vehicle2D( markerPosition( x0 ), radius=R/5, color='k',
        fig=fig, axs=axs, tail_length=1000, zorder=25 )

    # Initialize forward tail and plot.
    xpred = mpc_var.statePrediction( x0, uList )[:2,:]
    v_var.initForwardTail( xpred, zorder=20 )
    marker.linestyle = ':'  # manually set linestyle
    v_var.draw()
    marker.draw()

    # plt.axis( [-6, 6, -6, 6] )
    plt.gca().set_aspect( 'equal', adjustable='box' )
    plt.show( block=0 )

    # Simulation loop.
    x = x0
    u = uList
    input( "\nPress ENTER to start simulation loop..." )
    for i in range( Nt ):
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

        # Break if sim exceeds boundaries of T.
        if x[0] > T:
            break
    input( "\nPress ENTER to close program..." )

    # # Test path generator.
    # axs.plot( xList[0], xList[1],
    #     color='yellowgreen', marker='+' )
    # plt.show()
