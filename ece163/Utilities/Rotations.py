"""
Author : Anderson Compalas ( acompala@ucsc.edu )
This file contains tools to perform rotations by producing rotation matrixes .
"""

import math
from . import MatrixMath

def euler2DCM(yaw, pitch, roll):
    """
    Converts Euler angles (yaw, pitch, roll) to a Direction Cosine Matrix (DCM).
    
    The DCM represents the rotation matrix that can be used to transform a vector 
    from the body frame to the inertial frame.

    :param yaw: Rotation about the inertial Z-axis (in radians).
    :param pitch: Rotation about the intermediate Y-axis (in radians).
    :param roll: Rotation about the body X-axis (in radians).
    :return: A 3x3 rotation matrix (DCM) corresponding to the provided Euler angles.
    """
    return [
        [
            math.cos(pitch) * math.cos(yaw),
            math.cos(pitch) * math.sin(yaw),
            -math.sin(pitch)
        ],
        [
            math.sin(roll) * math.sin(pitch) * math.cos(yaw) - math.cos(roll) * math.sin(yaw),
            math.sin(roll) * math.sin(pitch) * math.sin(yaw) + math.cos(roll) * math.cos(yaw),
            math.sin(roll) * math.cos(pitch)
        ],
        [
            math.cos(roll) * math.sin(pitch) * math.cos(yaw) + math.sin(roll) * math.sin(yaw),
            math.cos(roll) * math.sin(pitch) * math.sin(yaw) - math.sin(roll) * math.cos(yaw),
            math.cos(roll) * math.cos(pitch)
        ]
    ]

def dcm2Euler(DCM):
    """
    Converts a Direction Cosine Matrix (DCM) to Euler angles (yaw, pitch, roll).
    
    The DCM is assumed to represent the rotation matrix transforming vectors from 
    the body frame to the inertial frame. This function extracts the corresponding 
    yaw, pitch, and roll angles.

    :param DCM: A 3x3 Direction Cosine Matrix.
    :return: A list [yaw, pitch, roll] where:
        - yaw: Rotation about the inertial Z-axis (in radians).
        - pitch: Rotation about the intermediate Y-axis (in radians).
        - roll: Rotation about the body X-axis (in radians).
    """
    # Extract pitch safely
    pitch = -math.asin(max(-1, min(1, DCM[0][2])))

    # Check for gimbal lock
    if abs(DCM[0][2]) != 1:
        roll = math.atan2(DCM[1][2], DCM[2][2])
        yaw = math.atan2(DCM[0][1], DCM[0][0])
    else:
        # Handle gimbal lock
        roll = math.atan2(DCM[1][2], DCM[2][2])
        if DCM[0][2] == -1:  # Facing straight down
            yaw = -roll
        else:  # Facing straight up
            yaw = roll

    # Normalize yaw to [-π, π]
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    
    # Temporary solution to pipeline
    # if math.isclose(yaw, -math.pi, abs_tol=1e-6) or math.isclose(yaw, math.pi, abs_tol=1e-6):
    #     yaw = 0
        
    # if (math.isclose(yaw, -math.pi, abs_tol=1e-6) or 
    #     math.isclose(yaw, math.pi, abs_tol=1e-6) or 
    #     math.isclose(yaw, math.pi / 2, abs_tol=1e-6) or 
    #     math.isclose(yaw, -math.pi / 2, abs_tol=1e-6)):
    #     yaw = 0

    return [yaw, pitch, roll]

def ned2enu(ned_points):
    """
    Converts points from the NED (North-East-Down) frame to the ENU (East-North-Up) frame.
    
    This transformation swaps the North and East axes and negates the Down axis, 
    making the output points compatible with ENU coordinate systems.

    :param ned_points: A single point or a list of points.
    :return: The corresponding point(s) in the ENU frame.
    """
    T_ned_to_enu = [
        [0, 1, 0],  # Swap North and East
        [1, 0, 0],  # Swap East and North
        [0, 0, -1]  # Negate Down to Up
    ]

    # Handle single NED point (3x1 column vector)
    if len(ned_points) == 3 and not isinstance(ned_points[0], list):
        # Ensure single NED point is flat
        transformed = MatrixMath.multiply(T_ned_to_enu, [[ned_points[0]], [ned_points[1]], [ned_points[2]]])
        return [t[0] for t in transformed]  # Flatten result

    # Handle multiple NED points (Nx3 row format)
    if isinstance(ned_points[0], list):
        return [[t[0] for t in MatrixMath.multiply(T_ned_to_enu, [[p[0]], [p[1]], [p[2]]])] for p in ned_points]

    # Invalid input format
    raise ValueError("Invalid input format for NED points")