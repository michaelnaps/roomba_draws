# System imports and path initializations.
import sys
from os.path import expanduser
sys.path.insert(0, expanduser('~')+'/prog/four')
sys.path.insert(0, expanduser('~')+'/prog/geom')

# Standard imports.
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# MPC class imports.
import GEOM.Vehicle2D as vhc
import FOUR.Transforms as four

dt = 0.001

# Main execution loop.
if __name__ == "__main__":
    # Import hat data and fit to necessary size.
    file = expanduser('~')+'/prog/four/abby_pkg/sketchdata.csv'
    data = pd.read_csv( file )
    xTrain = (1/20)*data[ data['z']=='d' ].to_numpy()[:,:2].T

    # Calculate Fourier transform over x-y sets.
    nTrain = xTrain.shape[1]
    tTrain = np.array( [[i*dt for i in range( nTrain )]] )
    fvar = four.RealFourier( tTrain, xTrain )
    fvar.ls( N=50 )

    # Simulation series.
    x0 = fvar.solve( np.array( [[0]] ) )
    T = 10;  Nt = round( T/dt ) + 1
    tList = np.array( [[i*dt for i in range( Nt )]] )
    pList = fvar.solve( tList )

    # Vehicle variable and static initializations.
    fig, axs = plt.subplots()
    axs.plot( pList[0], pList[1],
        color='r', linestyle='--', marker='x',
        markersize=2.5, label='Desired Path',
        zorder=10 )
    v_var = vhc.Vehicle2D( x0, radius=0.1,
        fig=fig, axs=axs, tail_length=1000, zorder=25 )

    # Initialize forward tail and plot.
    v_var.draw()

    # plt.axis( [-6, 6, -6, 6] )
    plt.gca().set_aspect( 'equal', adjustable='box' )
    plt.show( block=0 )

    # Simulation loop.
    x = x0
    input( "\nPress ENTER to start simulation loop..." )
    for t in tList.T:
        x = fvar.solve( t[:,None] )
        v_var.update( x )
        plt.pause( 1e-3 )

    input( "\nPress ENTER to close program..." )

    # # Test path generator.
    # axs.plot( xList[0], xList[1],
    #     color='yellowgreen', marker='+' )
    # plt.show()
