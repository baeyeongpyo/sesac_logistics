from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from foxglove_ros_worker.warehouse_zone_publisher import (
    build_marker_specs,
    load_warehouse_layout,
)


CONFIG_PATH = Path(__file__).resolve().parents[5] / 'config' / 'warehouse_zones.yaml'


class WarehouseZoneLayoutTest(unittest.TestCase):
    def test_current_map_layout_exposes_every_operator_supplied_center_point(self):
        """A missing or shifted warehouse point must be visible to the operator."""
        layout = load_warehouse_layout(CONFIG_PATH)

        self.assertEqual(layout.frame_id, 'map')
        self.assertEqual(layout.topic, '/warehouse/zones')
        self.assertEqual(layout.goal_center_offset_m, 0.15)
        self.assertEqual(
            [(point.id, point.center) for point in layout.points],
            [
                ('P1', (0.17, -2.56)),
                ('P2', (-0.08, -2.56)),
                ('P3', (-0.33, -2.56)),
                ('P4', (-0.58, -2.56)),
                ('F1', (0.19, -1.36)),
                ('F2', (0.19, -1.53)),
                ('F3', (0.19, -1.70)),
                ('F4', (0.00, -1.36)),
                ('F5', (0.00, -1.53)),
                ('F6', (0.00, -1.70)),
                ('F7', (-0.17, -1.36)),
                ('F8', (-0.17, -1.53)),
                ('F9', (-0.17, -1.70)),
                ('N1', (-0.75, 0.03)),
                ('N2', (-0.57, 0.03)),
                ('N3', (-0.39, 0.03)),
                ('N4', (-0.75, -0.15)),
                ('N5', (-0.57, -0.15)),
                ('N6', (-0.39, -0.15)),
                ('N7', (-0.75, -0.33)),
                ('N8', (-0.57, -0.33)),
                ('N9', (-0.39, -0.33)),
            ],
        )

    def test_layout_builds_a_dot_and_label_for_each_warehouse_point(self):
        """An operator point without its visible dot or label must be rejected."""
        specs = build_marker_specs(load_warehouse_layout(CONFIG_PATH))

        self.assertEqual(
            [(spec.id, spec.position) for spec in specs if spec.kind == 'warehouse_point'],
            [
                ('P1', (0.17, -2.56)), ('P2', (-0.08, -2.56)),
                ('P3', (-0.33, -2.56)), ('P4', (-0.58, -2.56)),
                ('F1', (0.19, -1.36)), ('F2', (0.19, -1.53)),
                ('F3', (0.19, -1.70)), ('F4', (0.00, -1.36)),
                ('F5', (0.00, -1.53)), ('F6', (0.00, -1.70)),
                ('F7', (-0.17, -1.36)), ('F8', (-0.17, -1.53)),
                ('F9', (-0.17, -1.70)), ('N1', (-0.75, 0.03)),
                ('N2', (-0.57, 0.03)), ('N3', (-0.39, 0.03)),
                ('N4', (-0.75, -0.15)), ('N5', (-0.57, -0.15)),
                ('N6', (-0.39, -0.15)), ('N7', (-0.75, -0.33)),
                ('N8', (-0.57, -0.33)), ('N9', (-0.39, -0.33)),
            ],
        )

    def test_layout_places_point_decals_below_vehicle_geometry(self):
        """A tall overlay marker must not cover a vehicle parked on a point."""
        specs = build_marker_specs(load_warehouse_layout(CONFIG_PATH))

        self.assertEqual(
            {(spec.z, spec.scale) for spec in specs if spec.kind == 'warehouse_point'},
            {(0.003, (0.06, 0.06, 0.006))},
        )
        self.assertEqual(
            {(spec.z, spec.scale) for spec in specs if spec.kind == 'warehouse_point_label'},
            {(0.012, (0.0, 0.0, 0.12))},
        )
        self.assertEqual(
            [(spec.id, spec.text) for spec in specs if spec.kind == 'warehouse_point_label'],
            [
                ('P1', 'P1'), ('P2', 'P2'), ('P3', 'P3'), ('P4', 'P4'),
                ('F1', 'F1'), ('F2', 'F2'), ('F3', 'F3'), ('F4', 'F4'),
                ('F5', 'F5'), ('F6', 'F6'), ('F7', 'F7'), ('F8', 'F8'),
                ('F9', 'F9'), ('N1', 'N1'), ('N2', 'N2'), ('N3', 'N3'),
                ('N4', 'N4'), ('N5', 'N5'), ('N6', 'N6'), ('N7', 'N7'),
                ('N8', 'N8'), ('N9', 'N9'),
            ],
        )


if __name__ == '__main__':
    unittest.main()
