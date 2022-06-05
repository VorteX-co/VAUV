#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat May  1 00:17:33 2021

@author: mohamedd
"""
from capytaine import FloatingBody
import numpy as np
import xarray as xr
from capytaine import *
import logging
import capytaine as cpt
from numpy import infty
from math import pi
from capytaine import Plane
import matplotlib.pyplot as plt
logging.basicConfig(level=logging.INFO)

body = FloatingBody.from_file('swift_simplified.stl')  # stl file
# Change Surge to Sway and Heave for y and z axis calculations
body.add_translation_dof(name="Surge")
"""
For rotational degree of freedom change the previous line
to body.add_roation_dof(name="Yaw")
"""
clipped_body = body.keep_immersed_part(sea_bottom=-infty, inplace=False)

test_matrix = xr.Dataset(coords={
    'omega': np.linspace(0.1, 8, 500),
    'radiating_dof': list(clipped_body.dofs.keys()),
})
ds2 = cpt.BEMSolver(green_function=cpt.XieDelhommeau()).fill_dataset(test_matrix, [clipped_body])
ds1 = cpt.BEMSolver(green_function=cpt.Delhommeau()).fill_dataset(test_matrix, [clipped_body])

plt.figure()
ds1['added_mass'].plot(x='omega', label='Delhommeau')
ds2['added_mass'].plot(x='omega', label='XieDelhommeau')
plt.legend()
plt.figure()
ds1['radiation_damping'].plot(x='omega', label='Delhommeau')
ds2['radiation_damping'].plot(x='omega', label='XieDelhommeau')
plt.legend()

plt.show()