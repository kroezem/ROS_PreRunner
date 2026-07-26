"""Unit tests for fixed-cardinality LaserScan rebinning."""

import math

import pytest
from runner_bringup.scan_rebinner import FULL_CIRCLE, rebin_scan
from sensor_msgs.msg import LaserScan


def make_scan(ranges, intensities=None):
    """Create an endpoint-inclusive full-circle scan for a test."""
    scan = LaserScan()
    scan.header.stamp.sec = 123
    scan.header.stamp.nanosec = 456789
    scan.header.frame_id = 'base_laser'
    scan.angle_min = 0.25
    scan.angle_max = 6.0
    scan.angle_increment = FULL_CIRCLE / (len(ranges) - 1)
    scan.time_increment = 0.0123
    scan.scan_time = 0.1
    scan.range_min = 0.02
    scan.range_max = 12.0
    scan.ranges = ranges
    scan.intensities = intensities or []
    return scan


def assert_pinned_metadata(output, bins):
    """Assert the fixed output geometry and copied source metadata."""
    assert output.angle_min == 0.0
    assert output.angle_max == pytest.approx(FULL_CIRCLE)
    assert output.angle_increment == pytest.approx(
        FULL_CIRCLE / (bins - 1))
    assert output.time_increment == pytest.approx(0.1 / (bins - 1))
    assert output.scan_time == pytest.approx(0.1)
    assert output.range_min == pytest.approx(0.02)
    assert output.range_max == pytest.approx(12.0)
    assert output.header.stamp.sec == 123
    assert output.header.stamp.nanosec == 456789
    assert output.header.frame_id == 'base_laser'


def test_fast_path_preserves_ranges_and_pins_metadata():
    scan = make_scan([1.0, 2.0, 3.0], [10.0])

    output = rebin_scan(scan, 3)

    assert list(output.ranges) == [1.0, 2.0, 3.0]
    assert list(output.intensities) == [10.0, 0.0, 0.0]
    assert len(output.intensities) == 3
    assert_pinned_metadata(output, 3)
    assert scan.angle_min == pytest.approx(0.25)
    assert scan.time_increment == pytest.approx(0.0123)


def test_downsampling_selects_nearest_angles_and_preserves_endpoints():
    scan = make_scan(
        [0.0, 1.0, 2.0, 3.0, 4.0, 5.0],
        [10.0, 11.0, 12.0, 13.0, 14.0, 15.0],
    )
    scan.angle_min = 0.0

    output = rebin_scan(scan, 4)

    assert list(output.ranges) == [0.0, 2.0, 3.0, 5.0]
    assert list(output.intensities) == [10.0, 12.0, 13.0, 15.0]
    assert output.ranges[0] == scan.ranges[0]
    assert output.ranges[-1] == scan.ranges[-1]


def test_downsampling_collision_tie_prefers_lower_source_index():
    scan = make_scan([0.0, 1.0, 2.0, 3.0])
    scan.angle_min = 0.0

    output = rebin_scan(scan, 3)

    assert list(output.ranges) == [0.0, 1.0, 3.0]


def test_upsampling_leaves_empty_bins_and_preserves_endpoints():
    scan = make_scan([1.0, 2.0, 3.0])
    scan.angle_min = 0.0

    output = rebin_scan(scan, 5)

    assert output.ranges[0] == 1.0
    assert math.isinf(output.ranges[1])
    assert output.ranges[2] == 2.0
    assert math.isinf(output.ranges[3])
    assert output.ranges[4] == 3.0
    assert list(output.intensities) == [0.0] * 5


def test_selected_nan_range_is_preserved():
    scan = make_scan([1.0, math.nan, 3.0])
    scan.angle_min = 0.0

    output = rebin_scan(scan, 5)

    assert math.isnan(output.ranges[2])


@pytest.mark.parametrize('intensities', [[], [7.0], [7.0, 8.0]])
def test_missing_or_short_intensities_are_normalized(intensities):
    scan = make_scan([1.0, 2.0, 3.0], intensities)
    scan.angle_min = 0.0

    output = rebin_scan(scan, 5)

    assert len(output.intensities) == 5
    assert output.intensities[0] == (
        intensities[0] if intensities else 0.0)
    assert output.intensities[2] == (
        intensities[1] if len(intensities) > 1 else 0.0)
    assert output.intensities[1] == 0.0
    assert output.intensities[3] == 0.0


def test_bins_below_two_are_rejected():
    with pytest.raises(ValueError, match='bins must be at least 2'):
        rebin_scan(make_scan([1.0, 2.0]), 1)


@pytest.mark.parametrize('ranges', [[], [1.0]])
def test_source_with_fewer_than_two_ranges_produces_no_output(ranges):
    scan = LaserScan()
    scan.ranges = ranges

    assert rebin_scan(scan, 503) is None
