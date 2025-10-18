import numpy as np
import math

def quaternion_from_euler(ai, aj, ak): 
    '''
    Arguments:
        ai, aj, ak: Euler angles in RPY order (Roll, Pitch and Yaw) (float)
    Returns:
        q: The corresponding quaternion in format [qx, qy, qz, qw] (python list)
    '''
    q = [0.0, 0.0, 0.0, 0.0]
    cy = math.cos(ak * 0.5)
    sy = math.sin(ak * 0.5)
    cp = math.cos(aj * 0.5)
    sp = math.sin(aj * 0.5)
    cr = math.cos(ai * 0.5)
    sr = math.sin(ai * 0.5)

    q[3] = cr * cp * cy + sr * sp * sy  # w
    q[0] = sr * cp * cy - cr * sp * sy  # x
    q[1] = cr * sp * cy + sr * cp * sy  # y
    q[2] = cr * cp * sy - sr * sp * cy  # z

    return q 


def lonlat2xyz(lat, lon, lat0, lon0): 
    # WGS84 ellipsoid constants:
    a = 6378137
    b = 6356752.3142
    e = math.sqrt(1-b**2/a**2)
    
    x = a*math.cos(math.radians(lat0))*math.radians(lon-lon0)/math.pow(1-e**2*(math.sin(math.radians(lat0)))**2,0.5)
    y = a*(1 - e**2)*math.radians(lat-lat0)/math.pow(1-e**2*(math.sin(math.radians(lat0)))**2,1.5)
    
    return x, y # x and y coordinates in a reference frame with the origin in lat0, lon0
