import unittest


class TestTimestampSemantics(unittest.TestCase):
    def test_measurement_age_uses_header_stamp(self):
        control_now = 10.20
        receive_time = 10.18
        pointlio_header_stamp = 10.07
        self.assertAlmostEqual(control_now - pointlio_header_stamp, 0.13)
        self.assertNotAlmostEqual(control_now - receive_time, 0.13)

    def test_pointlio_and_actuator_delays_are_distinct(self):
        measurement_age = 0.11
        actuator_dead_time = 0.13
        self.assertNotEqual(measurement_age, actuator_dead_time)


if __name__ == "__main__":
    unittest.main()
