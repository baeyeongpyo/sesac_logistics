import math
from mentorpi_scan_filter.filter_core import filter_ranges


def test_measured_fork_indices_are_filtered():
    filtered = filter_ranges([10.0] * 504, 30, 470)
    for index in (0, 30, 470, 503):
        assert math.isinf(filtered[index])
    assert filtered[31] == 10.0
    assert filtered[469] == 10.0


def test_second_range_extends_to_shorter_scan_end():
    filtered = filter_ranges([10.0] * 503, 30, 470)
    assert math.isinf(filtered[502])


def test_input_ranges_are_not_modified():
    ranges = [1.0, 2.0]
    filter_ranges(ranges, 0, 1)
    assert ranges == [1.0, 2.0]
