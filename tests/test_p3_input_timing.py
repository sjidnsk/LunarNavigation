"""P3 只读时序辅助的行为测试。"""
import pytest
import importlib.util
from pathlib import Path
_path = Path(__file__).resolve().parents[1] / "scripts/simulation/p3_input_timing.py"
assert _path.exists(), "P3 timing helper has not been implemented"
_spec = importlib.util.spec_from_file_location("p3_input_timing", _path)
_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_module)
record_sample = _module.record_sample
timing_health = _module.timing_health


def health(sample, ros=110_000_000_000, mono=11.0, **changes):
    options = dict(max_source_age_s=8.0, receive_timeout_s=6.0, progress_timeout_s=6.0)
    options.update(changes)
    return timing_health(sample, ros, mono, **options)


def test_delayed_source_reports_real_ages():
    sample = record_sample(None, 105_000_000_000, 110_000_000_000, 11.0)
    assert health(sample) == dict(ready=True, reason='READY', source_age_s=5.0,
                                 receive_age_s=0.0, progress_age_s=0.0)
    assert sample['source_ns'] == 105_000_000_000


def test_duplicates_refresh_receipt_without_reviving_frozen_source():
    first = record_sample(None, 105_000_000_000, 110_000_000_000, 11.0)
    sample = record_sample(first, 105_000_000_000, 117_000_000_000, 18.0)
    result = health(sample, 117_000_000_000, 18.0, max_source_age_s=20)
    assert result['reason'] == 'SOURCE_FROZEN'
    assert result['receive_age_s'] == 0
    assert result['progress_age_s'] == 7
    assert first['received_mono_s'] == 11


def test_source_rollback_keeps_high_watermark():
    sample = record_sample(None, 105_000_000_000, 110_000_000_000, 11.0)
    for source, receipt, mono in [(103, 111, 12), (104, 112, 13)]:
        sample = record_sample(sample, source * 10**9, receipt * 10**9, mono)
        assert health(sample, receipt * 10**9, mono)['reason'] == 'SOURCE_ROLLBACK'
        assert sample['last_advanced_mono_s'] == 11
    sample = record_sample(sample, 106_000_000_000, 113_000_000_000, 14)
    assert sample['last_advanced_mono_s'] == 14
    assert health(sample, 113_000_000_000, 14)['ready']


@pytest.mark.parametrize('ros,mono,reason', [
    (117_000_000_000, 18, 'RECEIVE_TIMEOUT'),
    (114_000_000_000, 15, 'SOURCE_TOO_OLD'),
    (109_000_000_000, 12, 'CLOCK_ROLLBACK'),
    (111_000_000_000, 10, 'CLOCK_ROLLBACK'),
    (120_000_000_000, 12, 'SOURCE_TOO_OLD'),
])
def test_stale_and_clock_faults_fail_closed(ros, mono, reason):
    sample = record_sample(None, 105_000_000_000, 110_000_000_000, 11)
    assert health(sample, ros, mono)['reason'] == reason
    assert not health(sample, ros, mono)['ready']


def test_missing_sample():
    assert health(None)['reason'] == 'MISSING_SAMPLE'


@pytest.mark.parametrize('source,receipt,mono,reason', [
    (0, 110_000_000_000, 11, 'INVALID_TIMESTAMP'),
    (-1, 110_000_000_000, 11, 'INVALID_TIMESTAMP'),
    (float('nan'), 110_000_000_000, 11, 'INVALID_TIMESTAMP'),
    (105_000_000_000, 0, 11, 'INVALID_TIMESTAMP'),
    (105_000_000_000, 110_000_000_000, float('inf'), 'INVALID_TIMESTAMP'),
    (111_000_000_000, 110_000_000_000, 11, 'FUTURE_TIMESTAMP'),
])
def test_invalid_and_future_samples(source, receipt, mono, reason):
    assert health(record_sample(None, source, receipt, mono))['reason'] == reason


@pytest.mark.parametrize('receipt,mono', [(109_000_000_000, 12), (111_000_000_000, 10)])
def test_receipt_clock_rollback_retained(receipt, mono):
    first = record_sample(None, 105_000_000_000, 110_000_000_000, 11)
    sample = record_sample(first, 106_000_000_000, receipt, mono)
    assert health(sample, receipt, mono)['reason'] == 'CLOCK_ROLLBACK'


@pytest.mark.parametrize('name', ['max_source_age_s', 'receive_timeout_s', 'progress_timeout_s'])
@pytest.mark.parametrize('value', [0, -1, float('nan'), float('inf'), True])
def test_budgets_must_be_finite_positive(name, value):
    with pytest.raises(ValueError):
        health(None, **{name: value})


def test_monitor_pause_cannot_extend_life():
    sample = record_sample(None, 105_000_000_000, 110_000_000_000, 11)
    assert health(sample)['ready']
    assert health(sample, 117_000_000_000, 18)['reason'] == 'RECEIVE_TIMEOUT'
    assert health(sample, 118_000_000_000, 19)['reason'] == 'RECEIVE_TIMEOUT'
    assert sample['received_mono_s'] == 11



def test_future_source_does_not_poison_progress_high_watermark():
    first = record_sample(None, 105_000_000_000, 110_000_000_000, 11)
    bad = record_sample(first, 999_000_000_000, 111_000_000_000, 12)
    assert health(bad, 111_000_000_000, 12)['reason'] == 'FUTURE_TIMESTAMP'
    recovered = record_sample(bad, 106_000_000_000, 112_000_000_000, 13)
    assert health(recovered, 112_000_000_000, 13)['ready']


def test_invalid_sample_cannot_erase_clock_rollback_latch():
    sample = record_sample(None, 105_000_000_000, 110_000_000_000, 11)
    sample = record_sample(sample, 106_000_000_000, 109_000_000_000, 12)
    sample = record_sample(sample, 0, 111_000_000_000, 13)
    sample = record_sample(sample, 107_000_000_000, 112_000_000_000, 14)
    assert health(sample, 112_000_000_000, 14)['reason'] == 'CLOCK_ROLLBACK'


def test_invalid_receipt_cannot_hide_clock_rollback():
    sample = record_sample(None, 105_000_000_000, 110_000_000_000, 11)
    sample = record_sample(sample, 0, 0, float('nan'))
    sample = record_sample(sample, 106_000_000_000, 109_000_000_000, 10)
    assert health(sample, 109_000_000_000, 10)['reason'] == 'CLOCK_ROLLBACK'


@pytest.mark.parametrize('ros,mono', [(float('nan'), 11), (110_000_000_000, float('inf'))])
def test_invalid_query_clock_rejected(ros, mono):
    sample = record_sample(None, 105_000_000_000, 110_000_000_000, 11)
    assert health(sample, ros, mono)['reason'] == 'INVALID_TIMESTAMP'
