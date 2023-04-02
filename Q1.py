# -*- coding: utf-8 -*-
"""
Created on Mon Jan 30 21:03:10 2023

@author: 91830
"""

import plane_truss_lib as tlib
import numpy as np
import math

def loadP(P):
    loads = [         [5, 0, 8e4],
                      [5, 1, -1e5],
                      [4, 1, -2e5],
                      [3, 1, -5e4],
                      [2, 1, -P]]
    return loads   

def printMemberForces(truss):
    for i in range(truss.n_members):
        id1, id2 = truss.members[i]
        print("Member : ({} ,{})".format(id1,id2))
        print("Internal Force", truss.member_forces[i])
        
def printDeflections(truss):
    deflection_matrix = np.zeros(truss.joints.shape)
    for i in range(truss.joints.shape[0]):
        deflection_matrix[i] = np.array([truss.dofs[2*i+0], truss.dofs[2*i+1]])
    print(deflection_matrix)

def angle(v1, v2):
    pass
    

def findAngle(truss):
    deflected_angles = np.zeros(truss.n_members)
    for i in range(truss.n_members):
        idx1, idx2 = truss.members[i]
        x1 = truss.joints[idx1, 0] + truss.dofs[2*idx1+0]
        x2 = truss.joints[idx2, 0] + truss.dofs[2*idx2+0]
        y1 = truss.joints[idx1, 1] + truss.dofs[2*idx1+1]
        y2 = truss.joints[idx2, 1] + truss.dofs[2*idx2+1]
        deflected_angles[i] = math.atan((y2-y1)/(x2-x1))
    return deflected_angles

def extremeP(A,truss,failure_strenth,max=False,min=False):
    P = 0
    if max:
        while True:
            truss.apply_loads(loadP(P))
            truss.solve()
            strength = truss.member_forces/A
            if(np.any(strength > failure_strenth)):
                break
            else:
                P += 1e3
        return P
    if min:
        while True:
            truss.apply_loads(loadP(P))
            truss.solve()
            strength = truss.member_forces/truss.As
            if(np.any(strength > failure_strenth)):
                break
            else: 
                P = P - 1e3
        return P
        
        
        

if __name__ == "__main__":
    joints1 = np.array([[0.0,0.0],
                       [2.0,0.0],
                       [4.0, 0.0],
                       [3.0, 2.0],
                       [1.0,2.0],
                       [0.0,2.0]])

    members1 = np.array([[0,1],
                        [0,4],
                        [0,5],
                        [1,2],
                        [1,3],
                        [1,4],
                        [2,3],
                        [3,4],
                        [4,5]
                  ], dtype=int)
    E = 2.1e11
    A = np.pi*(2.5*2.5)/1e4

    truss1 = tlib.PlaneTruss(joints1, members1, E, A)
    # truss.plot()

    constraints1 = [
                    [0, 0, 0.0],
                    [0, 1, 0.0],
                    [1, 1, 0.0]
                ]
    
    """ Q1. 
    Use the Python code for plane trusses to compute the internal forces for this
    truss.
    Yes the printed forces match with the one calculated by hand.
    """
    P = 1e4
    truss1.apply_constraints(constraints1)
    truss1.apply_loads(loadP(P))
    truss1.solve()
    printMemberForces(truss1)
    
    """ Q1 Part d.
    Use the code extremeP for calculating the max and min values of P.
    To find max value give the argument max = True.
    To find the minimum value give the argument min = True.
    """
    
    
    """ Q2 Part b
    """
    joints2 = np.array([[0,0],
                       [1,0],
                       [2,0],
                       [3,0],
                       [3,1],
                       [2,1],
                       [1,1],
                       [0,1]])
    
    members2 = np.array([[0,1],
                        [1,2],
                        [2,3],
                        [3,4],
                        [4,5],
                        [5,6],
                        [6,7],
                        [7,0],
                        [0,6],
                        [7,1],
                        [1,5],
                        [6,2],
                        [2,4],
                        [5,3]
                  ], dtype=int)
    
    E = 2.1e11
    A = np.pi*(2.5*2.5)/1e4
    
    truss2 = tlib.PlaneTruss(joints2, members2, E, A)
    # truss.plot()
    
    constraints2 = [
                    [0,0,0.0],
                    [0,1,0.0],
                    [3,0,0.0],
                    [3,1,0.0]
        ]
    truss2.apply_constraints(constraints2)
    
    loads2 = [
                [6,1,-100e3],
                [5,1,-200e3]
        ]
    truss2.apply_loads(loads2)
    truss2.solve()
    printMemberForces(truss2)
    
    printDeflections(truss2)
    
    print(truss2.reactions)
    print(truss2.thetas)
    print(findAngle(truss2))

    print(truss2.thetas - findAngle(truss2))
    
    print("Maximum P: "extremeP(A,truss1, 4e8, max=True))
    print("Minimum P: "extremeP(A,truss1, 4e8, min=True))
    
    

        

