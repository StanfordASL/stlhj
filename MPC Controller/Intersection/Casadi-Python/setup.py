import numpy as np

def load_params(hf):

    T = 1
    dt = 0.1
    iters = int(T/dt)

    return {'min':hf['g/min'][0], 'max':hf['g/max'][0], 'N':hf['g/N'][0], 'T':T,
            'dt':dt, 'iters':iters}
