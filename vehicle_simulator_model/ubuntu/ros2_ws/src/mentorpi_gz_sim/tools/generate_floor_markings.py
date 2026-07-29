#!/usr/bin/env python3
"""Generate deterministic, visual-only warehouse floor markings."""

import argparse
from pathlib import Path


GLYPHS = {
    'F': ('11111', '10000', '10000', '11110', '10000', '10000', '10000'),
    'R': ('11110', '10001', '10001', '11110', '10100', '10010', '10001'),
    'E': ('11111', '10000', '10000', '11110', '10000', '10000', '11111'),
    'S': ('01111', '10000', '10000', '01110', '00001', '00001', '11110'),
    'H': ('10001', '10001', '10001', '11111', '10001', '10001', '10001'),
    'N': ('10001', '11001', '11001', '10101', '10011', '10011', '10001'),
    'O': ('01110', '10001', '10001', '10001', '10001', '10001', '01110'),
    'M': ('10001', '11011', '10101', '10101', '10001', '10001', '10001'),
    'A': ('01110', '10001', '10001', '11111', '10001', '10001', '10001'),
    'L': ('10000', '10000', '10000', '10000', '10000', '10000', '11111'),
    'P': ('11110', '10001', '10001', '11110', '10000', '10000', '10000'),
    'I': ('11111', '00100', '00100', '00100', '00100', '00100', '11111'),
    'C': ('01111', '10000', '10000', '10000', '10000', '10000', '01111'),
    'D': ('11110', '10001', '10001', '10001', '10001', '10001', '11110'),
    '1': ('00100', '01100', '00100', '00100', '00100', '00100', '01110'),
    '2': ('01110', '10001', '00001', '00010', '00100', '01000', '11111'),
    '3': ('11110', '00001', '00001', '01110', '00001', '00001', '11110'),
    '4': ('00010', '00110', '01010', '10010', '11111', '00010', '00010'),
    ' ': ('00000', '00000', '00000', '00000', '00000', '00000', '00000'),
}

WHITE = '0.96 0.98 1.0 1'
GREEN = '0.08 0.58 0.20 0.62'
BLUE = '0.06 0.30 0.78 0.62'
YELLOW = '0.98 0.72 0.04 0.72'
RED = '0.86 0.08 0.08 0.72'
ORANGE = '0.96 0.44 0.04 0.68'
DARK = '0.10 0.12 0.14 0.92'


def _number(value):
    text = f'{value:.4f}'.rstrip('0').rstrip('.')
    return '0' if text == '-0' else text


def add_box(lines, name, x, y, size_x, size_y, color, *, yaw=0.0,
            center_z=0.001, size_z=0.002):
    lines.extend([
        f'      <visual name="{name}">',
        '        <pose>'
        f'{_number(x)} {_number(y)} {_number(center_z)} '
        f'0 0 {_number(yaw)}</pose>',
        '        <geometry>',
        '          <box>',
        f'            <size>{_number(size_x)} {_number(size_y)} '
        f'{_number(size_z)}</size>',
        '          </box>',
        '        </geometry>',
        '        <material>',
        f'          <ambient>{color}</ambient>',
        f'          <diffuse>{color}</diffuse>',
        '        </material>',
        '      </visual>',
    ])


def add_text(lines, name, text, center_x, center_y, pixel_size, color=WHITE):
    glyph_step = pixel_size * 6
    text_width = len(text) * glyph_step - pixel_size
    start_x = center_x - text_width / 2 + pixel_size / 2
    top_y = center_y + pixel_size * 3
    pixel_index = 0

    for glyph_index, character in enumerate(text):
        for row_index, row in enumerate(GLYPHS[character]):
            for column_index, enabled in enumerate(row):
                if enabled != '1':
                    continue
                add_box(
                    lines,
                    f'{name}_pixel_{pixel_index:03d}',
                    start_x + glyph_index * glyph_step
                    + column_index * pixel_size,
                    top_y - row_index * pixel_size,
                    pixel_size * 0.82,
                    pixel_size * 0.82,
                    color,
                    center_z=0.003,
                )
                pixel_index += 1


def add_zone_borders(lines, name, center_x, center_y, size_x, size_y, color):
    border = 0.045
    add_box(
        lines, f'{name}_north', center_x, center_y + size_y / 2,
        size_x, border, color, center_z=0.002,
    )
    add_box(
        lines, f'{name}_south', center_x, center_y - size_y / 2,
        size_x, border, color, center_z=0.002,
    )
    add_box(
        lines, f'{name}_east', center_x + size_x / 2, center_y,
        border, size_y, color, center_z=0.002,
    )
    add_box(
        lines, f'{name}_west', center_x - size_x / 2, center_y,
        border, size_y, color, center_z=0.002,
    )


def render():
    lines = [
        '<?xml version="1.0"?>',
        '<sdf version="1.8">',
        '  <model name="warehouse_markings">',
        '    <static>true</static>',
        '    <link name="markings">',
    ]

    add_box(lines, 'fresh_zone', -0.5, 0.7, 1.6, 1.0, GREEN)
    add_text(lines, 'fresh_label', 'FRESH', -0.5, 0.7, 0.075)

    add_box(lines, 'normal_zone', 3.45, 2.45, 1.7, 1.3, BLUE)
    add_text(lines, 'normal_label', 'NORMAL', 3.45, 2.45, 0.065)

    add_text(lines, 'pico_label', 'PICO', -0.2, 3.18, 0.09)
    for slot_index, slot_x in enumerate((-0.92, -0.2, 0.52), start=1):
        add_zone_borders(
            lines,
            f'pallet_zone_{slot_index}',
            slot_x,
            3.18,
            0.48,
            0.62,
            RED,
        )

    add_box(lines, 'road_north_line', 2.05, -2.2, 5.4, 0.055, YELLOW)
    add_box(lines, 'road_south_line', 2.05, -3.4, 5.4, 0.055, YELLOW)
    add_text(lines, 'road_2_label', 'ROAD 2', 0.05, -2.8, 0.085)

    for station, station_y in enumerate((2.3, 1.25, 0.20, -0.85), start=1):
        add_zone_borders(
            lines,
            f'workstation_{station}',
            -4.4,
            station_y,
            0.58,
            0.72,
            ORANGE,
        )
        add_text(
            lines,
            f'workstation_{station}_label',
            str(station),
            -4.4,
            station_y,
            0.085,
        )

    add_box(lines, 'charging_zone', 3.7, 0.35, 1.0, 1.1, YELLOW)
    add_box(
        lines, 'charging_bolt_upper', 3.78, 0.57,
        0.14, 0.42, DARK, yaw=-0.35, center_z=0.003,
    )
    add_box(
        lines, 'charging_bolt_middle', 3.68, 0.35,
        0.34, 0.14, DARK, yaw=-0.35, center_z=0.003,
    )
    add_box(
        lines, 'charging_bolt_lower', 3.60, 0.13,
        0.14, 0.42, DARK, yaw=-0.35, center_z=0.003,
    )

    lines.extend([
        '    </link>',
        '  </model>',
        '</sdf>',
        '',
    ])
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(
        description='Generate warehouse_markings/model.sdf.',
    )
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(render(), encoding='utf-8')


if __name__ == '__main__':
    main()
