# File: Plant.py
# Classes: Model()

import numpy as np

def TaylorMethod(F, x, u=[None], dt=1e-3):
    return x + dt*F( x, u )

class Model:
    def __init__(self, F, dt=1e-3,
            model_type='discrete'):
        # default to TaylorMethod when given model is continuous
        if model_type == 'continuous':
            self.model = lambda x,u=[None]: TaylorMethod( F, x, u, dt )
        else:
            self.model = F

        # set time-step for reference
        self.dt = dt

    def prop(self, x, u=[None]):
        return self.model( x, u )
