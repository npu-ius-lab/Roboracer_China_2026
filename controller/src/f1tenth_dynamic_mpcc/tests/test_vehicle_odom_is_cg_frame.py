import unittest
from pathlib import Path

class TestVehicleOdomIsCgFrame(unittest.TestCase):
    def test_controller_consumes_vehicle_centre_topic_without_second_transform(self):
        root = Path(__file__).resolve().parents[1]
        source = (root/'scripts/mpcc_node.py').read_text()
        self.assertIn('/localization/vehicle_odom', (root/'config/controller.yaml').read_text())
        self.assertNotIn('0.135 * math.cos', source)
        self.assertNotIn('0.135 * math.sin', source)
        self.assertNotIn('vehicle_base_to_body', source)

if __name__ == '__main__': unittest.main()
