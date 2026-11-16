"""Analytic frame/integration checks; synthetic captures are not hardware validation."""
import csv
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
from fixtures import attempt, wire_rows
from vofa_fusion import FusionBridge

spec=importlib.util.spec_from_file_location('replay_imu_x_mapping',ROOT/'scripts/replay-imu-x-mapping.py')
mapping=importlib.util.module_from_spec(spec)
spec.loader.exec_module(mapping)


class MappingReplayTests(unittest.TestCase):
    def test_only_initial_schema_zero_is_skipped_and_original_is_unchanged(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'capture.csv'
            valid=wire_rows([attempt(1),attempt(2)])
            with path.open('w',newline='',encoding='utf8') as stream:
                writer=csv.writer(stream)
                writer.writerow([f'I{i}' for i in range(96)])
                writer.writerows([[0]*96]+valid)
            original=path.read_bytes()
            rows,metadata=mapping.read_original(path)
            self.assertEqual([r['line'] for r in rows],[3,4])
            self.assertEqual(metadata['excluded'],[dict(line=2,reason='initial schema-zero row')])
            self.assertEqual(path.read_bytes(),original)
            with path.open('a',newline='',encoding='utf8') as stream:
                csv.writer(stream).writerow([0]*96)
            with self.assertRaisesRegex(ValueError,'line 5: unsupported schema 0'):
                mapping.read_original(path)

    def test_middle_attitude_and_x_reflection_restore_fixed_world_velocity(self):
        # At yaw +45 deg, old X reflection sends a true world +X velocity to -Y.
        rows=[dict(valid=8,imu_tick_ms=1000,imu_qw=math.cos(math.pi/8),imu_qx=0,
                   imu_qy=0,imu_qz=math.sin(math.pi/8),line=1),
              dict(valid=8,imu_tick_ms=1020,imu_qw=0,imu_qx=0,imu_qy=0,imu_qz=1,line=2)]
        q,source=mapping.Orientations(rows).at(1000)
        self.assertEqual(source,'exact')
        matrix=mapping.horizontal_matrix(q)
        corrected=mapping.changed_wheel(matrix,[0,-1])
        self.assertAlmostEqual(corrected[0],1,places=12)
        self.assertAlmostEqual(corrected[1],0,places=12)
        public=mapping.rotate(matrix,corrected,inverse=True)
        self.assertAlmostEqual(public[0],math.sqrt(.5),places=12)
        self.assertAlmostEqual(public[1],-math.sqrt(.5),places=12)

    def test_production_kf_replay_integrates_navigation_velocity_during_rotation(self):
        with tempfile.TemporaryDirectory() as directory:
            bridge=FusionBridge(Path(directory))
            try:
                state=bridge.new_state()
                rows=[]
                for i in range(161):
                    yaw=1.1*i*.02
                    row=attempt(i+1,tick=2000+i*20,initialized=i!=0,mount_qw=1,
                        middle_receiver_ms=2000+i*20,imu_tick_ms=2000+i*20,
                        imu_qw=math.cos(yaw/2),imu_qz=math.sin(yaw/2),
                        wheel_n_x_mps=-math.cos(2*yaw),wheel_n_y_mps=-math.sin(2*yaw))
                    output=bridge.update(state,row)
                    row['output_n_x_mps'],row['output_n_y_mps']=output['velocity_n_mps']
                    row['output_sample_us']=output['sample_us']
                    row['attempt_flags']=(15 if i else 7)|(16 if output['valid'] else 0)|(32 if output['fused'] else 0)
                    rows.append(row)
                baseline,trace,sources=mapping.replay(rows,bridge)
                self.assertEqual(baseline['validation']['known_exact']['mismatches'],0)
                self.assertEqual(baseline['validation']['known_exact']['calls'],161)
                self.assertEqual(sources,{'exact':161})
                self.assertTrue(all(t['candidate_fused'] for t in trace))
                distance=mapping.integral(trace,'candidate',2,5.2)
                self.assertAlmostEqual(distance[0],3.2,places=6)
                self.assertAlmostEqual(distance[1],0,places=6)
                cropped=mapping.integral(trace,'candidate',2.005,5.195)
                self.assertAlmostEqual(cropped[0],3.19,places=6)
                self.assertAlmostEqual(trace[10]['candidate_public'][0],math.cos(.22),places=6)
                self.assertAlmostEqual(trace[10]['candidate_public'][1],-math.sin(.22),places=6)
            finally:
                bridge.close()


if __name__=='__main__':
    unittest.main()
