"""Image-space diagnostics only; no hardware, TCP correction or pose generation."""
import numpy as np


def rigid(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(matrix[:3, :3].T@matrix[:3, :3], np.eye(3), atol=1e-7)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-7)):
        raise ValueError('Expected proper rigid transform')
    return matrix


def project(camera_from_flange, offset, intrinsics):
    fx, fy, cx, cy = np.asarray(intrinsics, dtype=float)
    if not np.isfinite([fx, fy, cx, cy]).all() or min(fx, fy) <= 0:
        raise ValueError('Invalid intrinsics')
    point = camera_from_flange@np.r_[np.asarray(offset, dtype=float), 1.]
    if not np.isfinite(point).all() or point[2] <= 0:
        raise ValueError('Reference projects behind camera')
    return np.array([fx*point[0]/point[2]+cx, fy*point[1]/point[2]+cy])


def sample_geometry(sample):
    transform = (np.linalg.inv(rigid(sample['candidate_provenance']['candidate_T_base_from_camera']))
                 @rigid(sample['robot_pose_record']['T_base_from_flange']))
    intrinsics = sample['rectified_left_intrinsics_fx_fy_cx_cy']
    # Check the source's stored projections against its stored transforms.
    for point in sample['points']:
        if point['pixel'] is not None:
            calculated = project(transform, point['flange_offset_xyz_m'], intrinsics)
            if not np.allclose(calculated, point['pixel'], atol=1e-4, rtol=0):
                raise ValueError('Stored projection differs from source transforms')
    return transform, intrinsics


def review_annotations(sample, logo_pixel, jaw_pixels):
    size = sample['image_resolution']
    pixels = np.asarray([logo_pixel, *jaw_pixels], dtype=float)
    if (pixels.shape != (3, 2) or not np.isfinite(pixels).all()
            or np.any(pixels < 0) or np.any(pixels[:, 0] >= size[0])
            or np.any(pixels[:, 1] >= size[1])):
        raise ValueError('Three finite original-resolution pixels inside the source image required')
    transform, intrinsics = sample_geometry(sample)
    logo = next(point for point in sample['points'] if point['name'] == 'LOGO fit')
    predicted_logo = project(transform, logo['flange_offset_xyz_m'], intrinsics)
    # Two points define the image of the 3D flange Z-axis. This is an infinite
    # image line, not a claim that a finite physical TCP is feasible there.
    a, b = [project(transform, [0, 0, distance], intrinsics) for distance in (0., .4)]
    vector = b-a
    length = np.linalg.norm(vector)
    midpoint = pixels[1:].mean(axis=0)
    result = {
        'type': 'historical_image_annotation_diagnostic',
        'operator_marked_logo_pixel': pixels[0].tolist(),
        'operator_marked_jaw_contact_pixels': pixels[1:].tolist(),
        'arithmetic_2d_jaw_midpoint_pixel': midpoint.tolist(),
        'predicted_logo_pixel': predicted_logo.tolist(),
        'logo_reprojection_error_px': float(np.linalg.norm(predicted_logo-pixels[0])),
        'projected_flange_axis_reference_pixels': [a.tolist(), b.tolist()],
        'axis_observability': 'degenerate' if length < 1e-6 else 'image_line_available',
        'jaw_midpoint_to_axis_line_px': None,
        'closest_axis_image_point': None,
        'hypothesis_to_marked_midpoint_px': [],
        'metric_error_mm': None, 'replacement_tcp_pose': None,
        'tcp_verified': False, 'camera_to_base_calibrated': False,
        'robot_execution_enabled': False, 'trajectory': None,
        'limitations': [
            'Operator clicks are approximate image annotations, not measured 3D references.',
            'The arithmetic mean of two projected pixels is not generally the projection of their 3D midpoint.',
            'Current jaw state need not match the opening state in CAD.',
            'Even a small image residual does not verify TCP depth or camera extrinsics.',
            'A line residual is diagnostic only; no automatic choice of axial length, transverse correction or tolerance pass.',
        ],
    }
    if length >= 1e-6:
        unit = vector/length
        foot = a+unit*np.dot(midpoint-a, unit)
        result['jaw_midpoint_to_axis_line_px'] = float(np.linalg.norm(midpoint-foot))
        result['closest_axis_image_point'] = foot.tolist()
    for point in sample['points']:
        if point['name'] == 'LOGO fit':
            continue
        pixel = project(transform, point['flange_offset_xyz_m'], intrinsics)
        result['hypothesis_to_marked_midpoint_px'].append({
            'name': point['name'], 'projected_pixel': pixel.tolist(),
            'distance_to_marked_midpoint_px': float(np.linalg.norm(pixel-midpoint))})
    return result
