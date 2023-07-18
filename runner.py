# System imports and path initializations.
import sys
from os.path import expanduser
sys.path.insert(0, expanduser('~')+'/prog/mpc')

# Standard import(s).
import numpy as np

# MPC class import(s).
import MPC.Plant as plant
import MPC.Optimizer as opt

# Hyper parameter(s)
dt = 0.02
P = 12
k = 2
R = 0.25
Nx = 3
Nu = 2
NAV = 0
MOB = 1


# Connect to IP address?
connect( '10.70.78.133:9999', '10.70.78.133:1234' )


# Model and cost declarations.
def model(x, u):
    xn = np.array( [
        x[0] + dt*np.cos(x[2])*(u[0] + u[1]),
        x[1] + dt*np.sin(x[2])*(u[0] + u[1]),
        x[2] + dt*1/R*(u[0] - u[1])
    ] )
    return xn

def cost(xList, uList):
    x0 = xList[:,0,None]
    pList = pathSteps( x0, beta=k*dt )

    C = [0]
    for x, p in zip( xList.T, pList.T ):
        C = C + (x[0] - p[0])**2 + (x[1] - p[1])**2

    return C


# Generate path list.
def path(xcoord):
    # ycoord = np.cos( xcoord )
    ycoord = 0.25*np.cos( xcoord ) \
        + 0.4*np.cos( xcoord )**2 \
        + np.cos( xcoord )**3 \
        + 0.1*np.cos( xcoord )**4
    # ycoord = np.sign( xcoord )
    return ycoord

def pathSteps(xstate, N=P+1, beta=dt):
    xcoord = xstate[0]
    pList = np.empty( (2, N) )
    for i in range( N ):
        ycoord = path( xcoord )
        pList[:,i] = np.hstack( (xcoord, ycoord) )
        xcoord = xcoord + beta
    return pList


# Motor control functions.
def stop_robot():
    command = 'motorctrl 0 0'
    send_command_to_robot(command, MOB)

def move_robot(u):
    command = 'motorctrl ' + str( u[0][0] ) + ' ' + str( u[0][0] )
    send_command_to_robot( command, MOB )


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
    mpc_var.setStepSize( 1.00 )

    # Generate the initial guess before the loop.
    uinit = np.zeros( (Nu,P) )
    mpc_var.setMaxIter( 1000 )
    uList = mpc_var.solve( x0, uinit, verbose=1 )
    mpc_var.setMaxIter( 10 )

    # Simulation loop.
    u = uList
    input( "\nPress ENTER to start runner loop..." )
    for i in range( Nt ):
        # Get current state value.
        x = get_pos().reshape(Nx,1)

        # Calculate optimal controls from MPC.
        u = mpc_var.solve( x, u, verbose=1 )

        # Update state and animation.
        move_robot( u )

        # Break if sim exceeds boundaries of T.
        if x[0] > T:
            stop_robot()
            break
    input( "\nPress ENTER to close program..." )
