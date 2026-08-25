#!/usr/bin/env python3
import unittest

from f1tenth_dynamic_mpcc import state_layout as layout
from f1tenth_dynamic_mpcc.acados_solver import AcadosDynamicMPCC
from f1tenth_dynamic_mpcc.vehicle_model import DynamicBicycleModel


class TestMPCCStateLayout(unittest.TestCase):
    def test_frozen_nine_state_three_input_layout(self):
        self.assertEqual(
            [layout.IDX_X, layout.IDX_Y, layout.IDX_PSI, layout.IDX_VX,
             layout.IDX_VY, layout.IDX_R, layout.IDX_DELTA,
             layout.IDX_DELTA_C, layout.IDX_THETA],
            list(range(9)),
        )
        self.assertEqual(
            [layout.IDX_V_CMD, layout.IDX_DELTA_C_RATE, layout.IDX_V_THETA],
            list(range(3)),
        )
        self.assertEqual((DynamicBicycleModel.NX, DynamicBicycleModel.NU), (9, 3))
        self.assertEqual((AcadosDynamicMPCC.NX, AcadosDynamicMPCC.NU), (9, 3))


if __name__ == "__main__":
    unittest.main()
