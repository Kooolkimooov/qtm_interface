#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.interpolate import CubicSpline




# def interpolation(waitpoints):

    # # Time intervall
    # t = np.linspace(-1,1,1000)

    # # indexs of points in time t intervall
    # step = round(len(t)/len(waitpoints[:,0]))
    # indexes = list(range(0,len(t),step))
    # print(indexes)

    # print()
    # print(t[indexes])

    # # Interpolation object (Boundary conditions) of X(t) and Y(t)
    # x_spline = CubicSpline(t[indexes],np.array(waitpoints[:,0]), bc_type="natural")
    # y_spline = CubicSpline(t[indexes],np.array(waitpoints[:,1]), bc_type="natural")

    # # x(t) and y(t) functions
    # x = x_spline(t)
    # y = y_spline(t)


    # plt.subplot(2,2,1)
    # plt.plot(waitpoints[:,0], waitpoints[:,1], "o", label="Data points")
    # plt.legend()

    # plt.subplot(2,2,2)
    # plt.plot(t, x, "red", label="x(t)")
    # plt.legend()

    # plt.subplot(2,2,3)
    # plt.plot(t, y, "red", label="y(t)")
    # plt.legend()

    # plt.subplot(2,2,4)
    # plt.plot(x, y, "red", label="Trajectory")
    # plt.legend()

    # plt.show()

def interpolation(waypoint_list, interpolation_number):
        waypoint_list = np.array(waypoint_list)
        # paramètre intermédiaire :
        t = np.linspace(0, 1, len(waypoint_list))
        tt = np.linspace(0, 1, interpolation_number)
        y_t = CubicSpline(t, waypoint_list[:, 1], bc_type="natural")
        y_tt = y_t(tt)
        x_t = CubicSpline(t, waypoint_list[:, 0], bc_type="natural")
        x_tt = x_t(tt)

        plt.plot(x_tt, y_tt, "red", label="Trajectory")
        plt.legend()
        plt.show()

        # return x_tt, y_tt



def main():
    waitpoints = np.array([[1, 1],[-1, 1], [-1, -1], [-0.4, -0.4], [1, -1]])
    interpolation(waitpoints,1000)




if __name__=="__main__":
    main()