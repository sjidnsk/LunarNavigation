"""Run with NAV_BENCHMARK=/absolute/path/to/lunar_adaptive_navigation_benchmark."""
import json,os,subprocess,unittest
class ComparisonContract(unittest.TestCase):
    def test_obstacle_changes_input_and_removal_restores_original_costs(self):
        binary=os.environ['NAV_BENCHMARK']
        rows=[json.loads(x) for x in subprocess.check_output([binary,'rough_cost','64','.2','13','1'],text=True).splitlines()]
        for algorithm in ['legacy_coarse','fine_reference','adaptive']:
            for scope in ['pipeline','search']:
                group={r['phase']:r for r in rows if r['algorithm']==algorithm and r['budget_scope']==scope}
                self.assertNotEqual(group['cold']['map_hash'],group['new_obstacle']['map_hash'])
                self.assertEqual(group['cold']['map_hash'],group['obstacle_removed']['map_hash'])
if __name__=='__main__':unittest.main()
