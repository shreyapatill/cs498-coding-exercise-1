import numpy as np
import math

def quaternion_from_euler(ai, aj, ak):
    '''
    Arguments:
        ai, aj, ak: Euler angles in RPY order (Roll, Pitch and Yaw) (float)
    Returns:
        q: The corresponding quaternion in format [qx, qy, qz, qw] (python list)
    '''
    # Half angles
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0

    # Compute sin and cos for each half angle
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)

    # Compute quaternion components
    qx = si * cj * ck - ci * sj * sk
    qy = ci * sj * ck + si * cj * sk
    qz = ci * cj * sk - si * sj * ck
    qw = ci * cj * ck + si * sj * sk

    q = [qx, qy, qz, qw]

    return q 


def lonlat2xyz(lat, lon, lat0, lon0): 
    # WGS84 ellipsoid constants:
    a = 6378137
    b = 6356752.3142
    e = math.sqrt(1-b**2/a**2)
    
    x = a*math.cos(math.radians(lat0))*math.radians(lon-lon0)/math.pow(1-e**2*(math.sin(math.radians(lat0)))**2,0.5)
    y = a*(1 - e**2)*math.radians(lat-lat0)/math.pow(1-e**2*(math.sin(math.radians(lat0)))**2,1.5)
    
    return x, y # x and y coordinates in a reference frame with the origin in lat0, lon0
