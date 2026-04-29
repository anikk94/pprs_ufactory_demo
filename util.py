import numpy as np
from scipy.spatial.transform import Rotation as R


def q2mat(q, scalar_last=True):
    """
    Convert a quaternion to a 3x3 rotation matrix.

    Args:
        q: Iterable of 4 elements. If scalar_last=True, order is (x, y, z, w),
           otherwise (w, x, y, z).
        scalar_last: Whether quaternion uses (x, y, z, w) ordering.

    Returns:
        3x3 numpy.ndarray rotation matrix.
    """
    q = np.asarray(q, dtype=float).reshape(4)
    if scalar_last:
        x, y, z, w = q
    else:
        w, x, y, z = q

    n = x * x + y * y + z * z + w * w
    if n == 0.0:
        raise ValueError("Zero-norm quaternion is not a valid rotation.")
    s = 2.0 / n

    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s

    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ]
    )



def URaa2rpy(axis,name=''):
    # axis = np.array([0.4251, 0.4275, -1.5247]) 
    angle = np.sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)
    # Normalize axis
    axis = axis / np.linalg.norm(axis)
    # Axis-angle to quaternion
    rotvec = axis * angle
    r = R.from_rotvec(rotvec)

    # Convert to roll (rx), pitch (ry), yaw (rp) in radians
    rx, ry, rp = r.as_euler('xyz', degrees=True)  # Use degrees=True for output in degrees

    print(f"{name}Roll (rx): {rx:.2f}°, Pitch (ry): {ry:.2f}°, Yaw (rp): {rp:.2f}°")


if __name__ == "__main__":

    # ps - plastic screw/printed screw
    # ph - plastic sphere
    # rs - real screw
    # rh - real sphere
    print("printed screws")
    URaa2rpy([0.4487, 0.457, -1.507]    , 'ps1 ')
    URaa2rpy([-0.5763, 0.3184, 2.087]   , 'ps2 ')
    URaa2rpy([-0.28, 1.5511, 0.2697]    , 'ps3 ')
    URaa2rpy([-0.0115, -0.0109, -1.5813], 'ps4 ')
    URaa2rpy([0.4447, -0.4421, 1.5289]  , 'ps5 ')
    print('spheres on printed screw base')
    URaa2rpy([-0.0126, 0.5391, -3.0524], 'ph1 ')
    URaa2rpy([-0.1099, 0.32, -2.4994]  , 'ph2 ')
    URaa2rpy([0.1483, 0.2217, -1.1968] , 'ph3 ')
    print('real screws')
    URaa2rpy([0.4261, 0.4283, -1.5248],  'rs1 ')
    URaa2rpy([0.4517, 1.4883, -0.4775],  'rs2 ')
    URaa2rpy([-0.5556, 0.3107, 2.0812],  'rs3 ')
    URaa2rpy([0.0022, 0.0032, -1.5527],  'rs4 ')
    URaa2rpy([0.4551, -0.4647, 1.5002],  'rs5 ')
    print('spheres on real screw base')
    URaa2rpy([0.2449, 0.6079, -0.757],   'rh1 ')
    URaa2rpy([-0.0539, 0.1707, 0.541],   'rh2 ')
    URaa2rpy([-0.0199, -0.4791, -0.05],  'rh3 ')

