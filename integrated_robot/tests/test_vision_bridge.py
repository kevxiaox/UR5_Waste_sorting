import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest

from sorting_fsm.cell import load_cell
from sorting_fsm.motion_planner import load_motion_config
from sorting_fsm.taught_route import TaughtRoute
from sorting_fsm.vision_bridge import (CameraToBase, FixedPlacement, MeasuredPlacement, VisionGate,
                                       make_vision_request, parse_detector_output)

ROOT = Path(__file__).resolve().parents[1]
POLICY = json.loads((ROOT / 'configs' / 'sorting_policy.json').read_text(encoding='utf-8'))
WORKSPACE = json.loads((ROOT / 'configs' / 'workspace_zones.json').read_text(encoding='utf-8'))
FIXTURES = ROOT / 'fixtures' / 'vision'
CELL = load_cell(ROOT / 'configs' / 'real_cell' / 'cell_config.json')
MOTION = load_motion_config(ROOT / 'configs' / 'real_cell' / 'motion_config.json')
ROUTE = TaughtRoute.from_cell(CELL, POLICY, MOTION['tcp_pose'], root=ROOT)
IDENTITY = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def item(name='sponge', category='General_Waste', conf=0.8, center=(850, 200), status='OK'):
    x, y = center
    return {'class_name': name, 'waste_category': category, 'confidence': conf,
            'bbox_xyxy': [x - 40, y - 35, x + 40, y + 35], 'center_pixel': [x, y], 'status': status}


def frame(*items, size=(1280, 720)):
    return json.dumps({'image_resolution': list(size), 'total_detected': len(items), 'items': list(items)})


def gate(frames_required=3):
    g = VisionGate(POLICY, WORKSPACE, frames_required=frames_required)
    g.confirm_camera_alignment()
    return g


class GateTests(unittest.TestCase):
    def test_stable_object_becomes_ready(self):
        g = gate()
        statuses = [g.observe(frame(item(center=(850 + i, 200))), i).status for i in range(1, 4)]
        self.assertEqual(statuses, ['collecting', 'collecting', 'ready'])

    def test_requires_operator_camera_confirmation(self):
        g = VisionGate(POLICY, WORKSPACE, frames_required=1)
        self.assertEqual(g.observe(frame(item()), 1).status, 'camera_not_confirmed')
        g.confirm_camera_alignment()
        self.assertEqual(g.observe(frame(item()), 2).status, 'ready')

    def test_policy_threshold_not_detector_threshold(self):
        result = gate(1).observe(frame(item(conf=0.5)), 1)
        self.assertEqual(result.status, 'no_candidate')
        self.assertIn('policy', result.rejected[0][2])

    def test_real_historical_detector_output_is_rejected(self):
        line = (FIXTURES / 'historical_low_confidence.jsonl').read_text(encoding='utf-8').splitlines()[0]
        result = gate(1).observe(line, 1)
        self.assertEqual(result.status, 'no_candidate')
        self.assertEqual({r[1] for r in result.rejected}, {'sponge', 'aluminum_can'})

    def test_two_objects_in_pick_zone_are_ambiguous(self):
        g = gate(1)
        result = g.observe(frame(item(), item('paper_cup', 'Recyclable', center=(880, 250))), 1)
        self.assertEqual(result.status, 'ambiguous')
        self.assertIsNone(result.candidate)

    def test_outside_pick_zone_category_mismatch_and_bad_status(self):
        g = gate(1)
        reasons = [g.observe(frame(case), i).rejected[0][2] for i, case in enumerate((
            item(center=(300, 300)), item(category='Recyclable'), item(status='REJECTED_LOW_CONF'),
            item('mug', 'UNKNOWN')), start=1)]
        self.assertEqual(reasons[0], 'outside PICK_ZONE')
        self.assertIn('category', reasons[1])
        self.assertIn('REJECTED_LOW_CONF', reasons[2])
        self.assertEqual(reasons[3], 'class not in the sorting policy')

    def test_policy_class_without_taught_plan_is_refused_at_placement(self):
        # toothbrush is in the policy (residual) but was never taught: no request, never a fallback.
        candidate = gate(1).observe(frame(item('toothbrush', 'General_Waste')), 1).candidate
        with self.assertRaisesRegex(ValueError, 'No taught references'):
            make_vision_request('sort-1', candidate, FixedPlacement(ROUTE), POLICY)

    def test_moving_or_changing_object_restarts_the_count(self):
        g = gate(3)
        g.observe(frame(item()), 1)
        g.observe(frame(item()), 2)
        self.assertEqual(g.observe(frame(item(center=(880, 240))), 3).stable_frames, 1)
        self.assertEqual(g.observe(frame(item('paper_cup', 'Recyclable', center=(880, 240))), 4).stable_frames, 1)
        self.assertEqual(g.observe(frame(), 5).status, 'no_candidate')
        self.assertEqual(g.observe(frame(item()), 6).stable_frames, 1)

    def test_malformed_detector_output_raises(self):
        g = gate(1)
        for bad in (frame(item(), size=(640, 480)),
                    json.dumps({'image_resolution': [1280, 720], 'total_detected': 2, 'items': [item()]}),
                    '{"image_resolution": [1280, 720], "total_detected": 1, "items": [{"confidence": NaN}]}'):
            with self.assertRaises(ValueError):
                g.observe(bad, 1)
        self.assertEqual(len(parse_detector_output(frame(item()), [1280, 720])), 1)


class PlacementTests(unittest.TestCase):
    def ready_candidate(self, name='plastic_bottle', category='Recyclable'):
        return gate(1).observe(frame(item(name, category)), 7).candidate

    def test_fixed_placement_uses_the_taught_spot_and_policy_zone(self):
        candidate = self.ready_candidate()
        request = make_vision_request('sort-1', candidate, FixedPlacement(ROUTE), POLICY)
        self.assertEqual(request.target_zone, 'LEFT_CELL')
        self.assertEqual(list(request.object_base_xyz_m), ROUTE.grasp_position('plastic_bottle'))
        self.assertIn('fixed_placement', request.source)

    def test_measured_placement_refuses_unvalidated_transform(self):
        with self.assertRaisesRegex(ValueError, 'not validated'):
            MeasuredPlacement(CameraToBase(IDENTITY, validated=False))

    def test_measured_placement_maps_camera_point(self):
        shift = [[1, 0, 0, 0.1], [0, 1, 0, -0.2], [0, 0, 1, 0.3], [0, 0, 0, 1]]
        placement = MeasuredPlacement(CameraToBase(shift, validated=True))
        request = make_vision_request('sort-2', self.ready_candidate(), placement, POLICY, (0.01, 0.02, 0.5))
        self.assertEqual([round(v, 9) for v in request.object_base_xyz_m], [0.11, -0.18, 0.8])
        for bad in (None, (0.0, 0.0, -1.0), (float('nan'), 0.0, 1.0)):
            with self.assertRaisesRegex(ValueError, 'depth'):
                placement.locate(self.ready_candidate(), bad)

    def test_transform_file_checks(self):
        path = Path(tempfile.mkdtemp()) / 't.json'
        path.write_text(json.dumps({'schema_version': 1, 'T_base_from_camera': IDENTITY}), encoding='utf-8')
        self.assertFalse(CameraToBase.load(path).validated)
        with self.assertRaisesRegex(ValueError, 'orthonormal'):
            CameraToBase([[2, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], validated=True)
        with self.assertRaisesRegex(ValueError, 'proper'):
            CameraToBase([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], validated=True)


class EndToEndTests(unittest.TestCase):
    def run_sorting(self, *args):
        sys.path.insert(0, str(ROOT))
        import run_sorting
        journal = Path(tempfile.mkdtemp()) / 'j.jsonl'
        saved = sys.argv
        sys.argv = ['run_sorting.py', *args, '--journal', str(journal)]
        out = io.StringIO()
        try:
            with contextlib.redirect_stdout(out):
                code = run_sorting.main()
        finally:
            sys.argv = saved
        lines = journal.read_text(encoding='utf-8').splitlines() if journal.exists() else []
        return code, out.getvalue(), [json.loads(line) for line in lines]

    def test_replayed_detections_sort_one_sponge(self):
        code, out, journal = self.run_sorting('--replay', str(FIXTURES / 'sponge_stable.jsonl'))
        self.assertEqual(code, 0, out)
        completed = [e for e in journal if e.get('outcome') == 'COMPLETED']
        self.assertEqual(len(completed), 1)
        self.assertEqual(completed[0]['details']['target_zone'], 'RIGHT_CELL')

    def test_ambiguous_scene_sends_nothing(self):
        code, out, journal = self.run_sorting('--replay', str(FIXTURES / 'two_objects_ambiguous.jsonl'))
        self.assertEqual(code, 1)
        self.assertIn('ambiguous', out)
        self.assertFalse(any(e.get('event') == 'command_submitted' for e in journal))


if __name__ == '__main__':
    unittest.main()
