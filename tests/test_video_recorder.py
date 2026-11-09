from unittest.mock import Mock

import pytest

from emdb_simulator.core.video_recorder import EpisodeRecordingSpec


@pytest.mark.parametrize("spec", ["all", "ALL", " * "])
def test_match_all(spec):
    s = EpisodeRecordingSpec.parse(spec)
    assert s.contains(0) and s.contains(10_000)


def test_ids_and_ranges():
    s = EpisodeRecordingSpec.parse("0-2, 5 ,10-12")
    assert [i for i in range(15) if s.contains(i)] == [0, 1, 2, 5, 10, 11, 12]


@pytest.mark.parametrize("spec", [None, "", "   ", "abc", "5-2", "1-x"])
def test_bad_or_empty_spec_records_nothing(spec):
    assert not any(EpisodeRecordingSpec.parse(spec).contains(i) for i in range(20))


def test_bad_tokens_are_skipped_and_logged():
    logger = Mock()
    s = EpisodeRecordingSpec.parse("3,oops,7-6,9", logger=logger)
    assert [i for i in range(12) if s.contains(i)] == [3, 9]
    assert logger.warning.call_count == 2
