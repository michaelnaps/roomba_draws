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
import GEOM.Vectors as vect
import FOUR.Transforms as four

dt = 0.25
W = 1

# Main execution loop.
if __name__ == "__main__":
    # Import hat data and fit to necessary size.
    file = expanduser('~')+'/prog/four/abby_pkg/sketchdata.csv'
    data = pd.read_csv( file )
    xTrain = data[ data['z']=='d' ].to_numpy()[::15,:2].T

    # Calculate Fourier transform over x-y sets.
    nTrain = xTrain.shape[1]
    tTrain = W*np.array( [[i for i in range( nTrain )]] )
    fvar = four.RealFourier( tTrain, xTrain )
    fvar.dmd( N=25 )

    # Simulation series.
    t0 = np.array( [[0]] )
    x0 = fvar.solve( t0 )
    Nt = round( nTrain/dt )
    tList = np.array( [[i*dt for i in range( Nt )]] )
    pList = fvar.solve( tTrain )

    print( 'x:', min( xTrain[0] ), max( xTrain[0] ) )
    print( 'y:', min( xTrain[1] ), max( xTrain[1] ) )

    ox = np.array( [[0],[-25]] )
    oy = np.array( [[-125],[0]] )

    # Vehicle variable and static initializations.
    fig, axs = plt.subplots()
    axs.plot( xTrain[0], xTrain[1],
        color='r', linestyle='None', marker='x',
        markersize=2.5, label='Desired Path',
        zorder=10 )
    v_var = vhc.Vehicle2D( x0, radius=0.1,
        fig=fig, axs=axs, tail_length=1000, zorder=25 )

    # Draw Fourier vectors.
    fvect = fvar.vectors( t0 )
    xvect = vect.Vectors( fvect[0]+ox, fig=fig, axs=axs, color='k' )
    dxvect = vect.Vectors( np.hstack( (fvect[0][:,-1,None]+ox, x0) ),
        fig=fig, axs=axs, color='grey' )
    yvect = vect.Vectors( np.flipud( fvect[1] )+oy, fig=fig, axs=axs, color='k' )
    dyvect = vect.Vectors( np.hstack( (np.flipud( fvect[1][:,-1,None] )+oy, x0) ),
        fig=fig, axs=axs, color='grey' )

    # Initialize forward tail and plot.
    axs.axes.xaxis.set_ticklabels( [] )
    axs.axes.yaxis.set_ticklabels( [] )
    v_var.draw()
    xvect.draw()
    dxvect.setLineStyle( ':' );  dxvect.draw()
    yvect.draw()
    dyvect.setLineStyle( ':' );  dyvect.draw()

    plt.axis( [-250, 400, -150, 400] )
    plt.gca().set_aspect( 'equal', adjustable='box' )
    plt.show( block=0 )

    # Simulation loop.
    x = x0
    input( "\nPress ENTER to start simulation loop..." )
    for t in tList.T:
        x = fvar.solve( t[:,None] )
        fvect = fvar.vectors( t[:,None] )

        xvect.setVertices( fvect[0]+ox )
        dxvect.setVertices( np.hstack( (fvect[0][:,-1,None]+ox, x) ) )
        yvect.setVertices( np.flipud( fvect[1] )+oy )
        dyvect.setVertices( np.hstack( ( np.flipud( fvect[1][:,-1,None] )+oy, x) ) )

        v_var.update( x )
        xvect.update()
        dxvect.update()
        yvect.update()
        dyvect.update()

        plt.pause( 1e-3 )

    input( "\nPress ENTER to close program..." )

    # # Test path generator.
    # axs.plot( xList[0], xList[1],
    #     color='yellowgreen', marker='+' )
    # plt.show()
