"""Selected Q0-Preview geometry helpers for native Panthera MuJoCo TCP frames.
Derived from XPolicyLab/Q0_Preview (Apache-2.0), commit
6baccd57ff958570d0db1588370289ee36c8bb70. No controller or simulator stepping.
See third_party/q0_preview/upstream/LICENSE and docs/q0_preview/MIGRATION.md.
"""
import numpy as np
from scipy.spatial.transform import Rotation

def at_height(origin, direction, z):
    """World point where a forward ray reaches z; reject parallel/backward intersections."""
    origin, direction = _vec3(origin, 'origin'), _vec3(direction, 'direction')
    z = _scalar(z, 'z (world metres)')
    if abs(direction[2]) < 1e-10:
        raise ValueError('Ray is parallel to the requested height plane')
    distance = (z - origin[2]) / direction[2]
    if distance < 0:
        raise ValueError('Height plane intersects behind the ray origin')
    return origin + distance * direction

def triangulate(p1, d1, p2, d2):
    """Midpoint of the closest approach of two rays and their miss distance."""
    p1, d1, p2, d2 = [_vec3(v, name) for v, name in zip((p1,d1,p2,d2), ('p1','d1','p2','d2'))]
    if np.linalg.norm(np.cross(d1, d2)) <= 1e-10*np.linalg.norm(d1)*np.linalg.norm(d2):
        raise ValueError('Triangulation needs two nonzero, nonparallel camera rays')
    s = np.linalg.lstsq(np.c_[d1, -d2], p2 - p1, rcond=None)[0]
    q1, q2 = p1 + s[0] * d1, p2 + s[1] * d2
    return (q1 + q2) / 2, float(np.linalg.norm(q1 - q2))

def _numeric_array(value, shape, name):
    try:
        result = np.asarray(value)
    except (TypeError, ValueError):
        result = None
    description = '3-vector [x,y,z]' if shape == (3,) else 'array of shape '+str(shape)
    if (result is None or result.shape != shape or result.dtype.kind not in 'iuf'
            or not np.isfinite(result).all()):
        raise ValueError(f'{name} must be a finite numeric {description}')
    return result.astype(float)

def _vec3(value, name):
    return _numeric_array(value, (3,), name)

def _scalar(value, name):
    if isinstance(value, (bool, np.bool_)) or not np.isscalar(value) or not np.isreal(value):
        raise ValueError(f'{name} must be one finite number')
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError(f'{name} must be one finite number') from None
    if not np.isfinite(result):
        raise ValueError(f'{name} must be one finite number')
    return result

def _target(target):
    from copy import deepcopy
    if not isinstance(target, dict):
        raise ValueError('target must be a single-arm action dict with target_points and orientation_point')
    required = {'target_points', 'orientation_point'}
    if not required <= target.keys() or target.keys() - required - {'contact_constraint'}:
        raise ValueError('target needs target_points and orientation_point, with optional contact_constraint; '
                         'convert an observed arm with geom.target_from_state(arm_state)')
    result = deepcopy(target)
    result['target_points'] = _numeric_array(target['target_points'], (2, 3), 'target_points').tolist()
    result['orientation_point'] = _vec3(target['orientation_point'], 'orientation_point').tolist()
    return result

def target_from_state(arm_state):
    """Single observed arm -> action dict. Preserve point order; put o 0.1 m behind midpoint.

    Pass obs.state()['state']['left'] or ['right'], not the complete observation.
    Does not infer gripper commands or contact constraints.
    """
    if not isinstance(arm_state, dict) or not {'gripper_pose_points', 'tool_axis'} <= arm_state.keys():
        raise ValueError("arm_state must be obs.state()['state']['left'] or ['right']")
    points = _numeric_array(arm_state['gripper_pose_points'], (2, 3), 'gripper_pose_points')
    tool = _vec3(arm_state['tool_axis'], 'tool_axis')
    if np.linalg.norm(tool) < 1e-10:
        raise ValueError('tool_axis must be nonzero')
    return {'target_points': points.tolist(),
            'orientation_point': (points.mean(axis=0)-.1*tool/np.linalg.norm(tool)).tolist()}

def translate_points(target, displacement):
    """Action dict -> new dict; translate three points in world metres, keep other fields."""
    target = _target(target)
    d = _vec3(displacement, 'displacement')
    target['target_points'] = [(np.asarray(p)+d).tolist() for p in target['target_points']]
    target['orientation_point'] = (np.asarray(target['orientation_point'])+d).tolist()
    return target

def rotate_points(target, pivot, rotation_vector):
    """New action dict rotated about a world pivot; rotation_vector is axis-angle radians.

    Vector direction is the world rotation axis; its norm is the angle, not Euler angles.
    Point order and other fields, including any fixed-world contact plane, are retained.
    """
    target = _target(target)
    r, p = Rotation.from_rotvec(_vec3(rotation_vector, 'rotation_vector')), _vec3(pivot, 'pivot')
    target['target_points'] = [(p+r.apply(np.asarray(q)-p)).tolist() for q in target['target_points']]
    target['orientation_point'] = (p+r.apply(np.asarray(target['orientation_point'])-p)).tolist()
    return target


def target_to_tcp(target):
    """Decode three ordered virtual anchors to the native TCP pose; not jaw opening.

    Axes are native TCP +X (tool), +Y (jaw reference), +Z. Anchors do not
    claim physical finger contact positions or an ARX-to-Panthera calibration.
    """
    target = _target(target)
    points = np.asarray(target['target_points'])
    center = points.mean(0)
    tool = center-np.asarray(target['orientation_point'])
    if np.linalg.norm(tool)<1e-8:
        raise ValueError('orientation point coincides with midpoint')
    tool /= np.linalg.norm(tool)
    jaw = points[1]-points[0]
    jaw -= tool*np.dot(jaw,tool)
    if np.linalg.norm(jaw)<1e-8:
        raise ValueError('ordered anchors are coincident or collinear with tool axis')
    jaw /= np.linalg.norm(jaw)
    pose=np.eye(4);pose[:3,3]=center;pose[:3,:3]=np.column_stack([tool,jaw,np.cross(tool,jaw)])
    return pose


def target_from_tcp(pose, reference_span_m=.04):
    pose=_numeric_array(pose,(4,4),'TCP pose')
    r=pose[:3,:3]
    if not np.allclose(pose[3],[0,0,0,1]) or not np.allclose(r.T@r,np.eye(3),atol=1e-7) or not np.isclose(np.linalg.det(r),1):
        raise ValueError('TCP pose must be a rigid transform')
    span=_scalar(reference_span_m,'reference span')
    if span<=0:raise ValueError('reference span must be positive')
    p=pose[:3,3]
    return dict(target_points=[(p-span*.5*r[:,1]).tolist(),(p+span*.5*r[:,1]).tolist()],orientation_point=(p-.1*r[:,0]).tolist())


def grasp_reference_for(working_point, tool_axis, jaw_axis, working_point_local):
    """Requires an explicitly measured Panthera offset; no X5 default dimensions."""
    p=_vec3(working_point,'working point');t=_vec3(tool_axis,'tool axis');j=_vec3(jaw_axis,'jaw axis')
    if np.linalg.norm(t)<1e-8:raise ValueError('zero tool axis')
    t/=np.linalg.norm(t);j-=t*(t@j)
    if np.linalg.norm(j)<1e-8:raise ValueError('collinear jaw axis')
    j/=np.linalg.norm(j)
    return p-np.column_stack([t,j,np.cross(t,j)])@_vec3(working_point_local,'measured local working offset')


def _camera_matrices(K, camera_to_world):
    K=_numeric_array(K,(3,3),'K');T=_numeric_array(camera_to_world,(4,4),'camera_to_world')
    target_from_tcp(T)
    if K[0,0]<=0 or K[1,1]<=0 or abs(np.linalg.det(K))<1e-12 or not np.allclose(K[2],[0,0,1]):
        raise ValueError('K must be nonsingular pinhole intrinsics with positive focal lengths')
    return K,T


def camera_ray(K, camera_to_world, u, v):
    """Explicit, same-frame calibration; OpenGL +X right,+Y up,-Z forward."""
    K,T=_camera_matrices(K,camera_to_world)
    uv=[_scalar(u,'u'),_scalar(v,'v'),1.]
    return T[:3,3].copy(),T[:3,:3]@(np.linalg.solve(K,uv)*[1,-1,-1])


def project_camera(K, camera_to_world, point):
    K,T=_camera_matrices(K,camera_to_world)
    optical=(T[:3,:3].T@(_vec3(point,'point')-T[:3,3]))*[1,-1,-1]
    if optical[2]<=0:return None
    pixel=K@optical
    return pixel[:2]/pixel[2]
