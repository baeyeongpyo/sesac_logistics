"""LaserScan 인덱스 기반 필터 계산."""

import math

DEFAULT_FIRST_END_INDEX = 30
DEFAULT_SECOND_START_INDEX = 470


def filter_ranges(ranges, first_end_index=DEFAULT_FIRST_END_INDEX,
                  second_start_index=DEFAULT_SECOND_START_INDEX):
    """0..first_end와 second_start..끝 구간을 inf로 바꾼다."""
    if first_end_index < 0:
        raise ValueError("first_end_index must be non-negative")
    if second_start_index <= first_end_index:
        raise ValueError("second_start_index must be greater than first_end_index")

    filtered = list(ranges)
    first_stop = min(first_end_index + 1, len(filtered))
    for index in range(first_stop):
        filtered[index] = math.inf
    for index in range(second_start_index, len(filtered)):
        filtered[index] = math.inf
    return filtered
