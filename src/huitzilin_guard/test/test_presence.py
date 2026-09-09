"""The alarm decision: when it sounds, when it stops, and when it gives up.

Runs without ROS:
    python -m pytest src/huitzilin_guard/test/test_presence.py

Time is passed in explicitly, so every window below is exercised
deterministically and none of these tests sleeps.
"""

import pytest

from huitzilin_guard.presence import PresenceLatch, PresencePolicy

POLICY = PresencePolicy(confirm_frames=3, clear_after_s=3.0,
                        min_alert_s=3.0, max_alert_s=300.0,
                        stale_input_s=5.0)


def sound_the_alarm(latch, start_s=0.0):
    """Drive the latch to ON through its confirm window. Returns the time."""
    now = start_s
    for _ in range(latch.policy.confirm_frames - 1):
        assert latch.on_frame(True, now) is None
        now += 0.1
    assert latch.on_frame(True, now) is True
    return now


def test_one_in_box_detection_does_not_sound_the_alarm():
    """A single frame is a detector artefact often enough that a one-frame
    trigger would make the siren untrustworthy."""
    latch = PresenceLatch(POLICY)
    assert latch.on_frame(True, 0.0) is None
    assert latch.is_on is False


def test_three_consecutive_in_box_detections_sound_it():
    latch = PresenceLatch(POLICY)
    assert latch.on_frame(True, 0.0) is None
    assert latch.on_frame(True, 0.1) is None
    assert latch.on_frame(True, 0.2) is True
    assert latch.is_on is True


def test_the_confirm_run_must_be_consecutive():
    """Two detections either side of an empty frame are not a confirmation;
    otherwise scattered false positives would accumulate into an alarm."""
    latch = PresenceLatch(POLICY)
    latch.on_frame(True, 0.0)
    latch.on_frame(True, 0.1)
    assert latch.on_frame(False, 0.2) is None
    assert latch.on_frame(True, 0.3) is None
    assert latch.is_on is False


def test_a_sounding_alarm_returns_none_rather_than_re_driving_the_output():
    latch = PresenceLatch(POLICY)
    now = sound_the_alarm(latch)
    assert latch.on_frame(True, now + 0.1) is None


def test_one_empty_frame_does_not_silence_a_sounding_alarm():
    """This is the case the detector is worst at: a person standing still.
    A single missed frame must never stop a correct alarm."""
    latch = PresenceLatch(POLICY)
    now = sound_the_alarm(latch)
    assert latch.on_frame(False, now + 0.1) is None
    assert latch.on_tick(now + 0.2) is None
    assert latch.is_on is True


def test_the_alarm_stops_after_the_box_has_been_clear_long_enough():
    """Frames keep ARRIVING and keep showing nobody, which is what an
    all-clear actually is. The last in-box detection was at 0.2 s, so the
    3 s clear window closes at 3.2 s."""
    latch = PresenceLatch(POLICY)
    last_seen = sound_the_alarm(latch)
    assert last_seen == pytest.approx(0.2)

    for t in (1.0, 2.0, 3.0):
        latch.on_frame(False, t)
        assert latch.on_tick(t) is None

    latch.on_frame(False, 3.2)
    assert latch.on_tick(3.2) is False
    assert latch.is_on is False
    assert "clear" in latch.last_stop_reason


def test_an_empty_box_is_only_believed_when_a_frame_actually_reports_it():
    """Elapsed time is not evidence. If the detector dies while the alarm is
    sounding, no frame ever observes an empty box, so the alarm must NOT
    stop with the reason "box clear" -- that would be a measurement nothing
    made. It runs on to the dead-man instead, which says what really
    happened.
    """
    latch = PresenceLatch(POLICY)
    sound_the_alarm(latch)                    # last in-box detection at 0.2

    # Past the 3 s clear window, but not one frame has arrived since.
    assert latch.on_tick(3.3) is None
    assert latch.on_tick(4.0) is None
    assert latch.is_on is True

    assert latch.on_tick(5.2) is False
    assert "no detection frame" in latch.last_stop_reason
    assert "clear" not in latch.last_stop_reason


def test_a_brief_intrusion_still_produces_a_perceptible_signal():
    """min_alert_s. Somebody who steps in and straight back out must still
    set off something a person can hear."""
    latch = PresenceLatch(PresencePolicy(confirm_frames=1, clear_after_s=0.5,
                                         min_alert_s=3.0, max_alert_s=300.0,
                                         stale_input_s=5.0))
    assert latch.on_frame(True, 0.0) is True
    latch.on_frame(False, 0.1)
    # The box has been clear well past clear_after_s, but not past
    # min_alert_s, so the siren is still sounding.
    assert latch.on_tick(1.0) is None
    assert latch.on_tick(2.9) is None
    assert latch.is_on is True
    assert latch.on_tick(3.0) is False


def test_a_stuck_detector_cannot_hold_the_siren_on_forever():
    """max_alert_s. Long on purpose, so a real intruder standing still
    cannot outwait it."""
    latch = PresenceLatch(POLICY)
    now = sound_the_alarm(latch)
    t = now
    while t < now + 299.0:
        t += 1.0
        latch.on_frame(True, t)
        assert latch.on_tick(t) is None
    assert latch.on_tick(now + 300.0) is False
    assert "max_alert_s" in latch.last_stop_reason


def test_the_stuck_detector_guard_outranks_the_minimum_duration():
    """A minimum audible duration must never be able to extend a runaway."""
    policy = PresencePolicy(confirm_frames=1, clear_after_s=1.0,
                            min_alert_s=2.0, max_alert_s=3.0,
                            stale_input_s=10.0)
    latch = PresenceLatch(policy)
    assert latch.on_frame(True, 0.0) is True
    latch.on_frame(True, 2.9)
    assert latch.on_tick(3.0) is False


def test_a_sounding_alarm_gives_up_when_its_frames_stop_arriving():
    """stale_input_s is a dead-man: a siren meaning "I have no idea" is
    worse than silence."""
    latch = PresenceLatch(POLICY)
    now = sound_the_alarm(latch)
    assert latch.on_tick(now + 4.9) is None
    assert latch.is_on is True
    assert latch.on_tick(now + 5.0) is False
    assert "no detection frame" in latch.last_stop_reason


def test_silence_on_an_empty_box_is_not_treated_as_a_fault():
    """The detector publishes only when it sees somebody, so an empty box and
    a dead detector both look like silence. Escalating that would leave the
    system permanently faulted on any quiet night, which is how a real fault
    gets trained out of an operator's attention."""
    latch = PresenceLatch(POLICY)
    for t in range(0, 60):
        assert latch.on_tick(float(t)) is None
    assert latch.is_on is False
    assert latch.seconds_since_input(60.0) is None


def test_there_is_no_cooldown_after_an_alarm_clears():
    """Someone still in the box is still an intruder. A quiet period after a
    clear would be a window in which the alarm deliberately does not work."""
    latch = PresenceLatch(POLICY)
    now = sound_the_alarm(latch)
    t = now + 4.0
    latch.on_frame(False, t)
    assert latch.on_tick(t) is False

    t += 0.1
    assert latch.on_frame(True, t) is None
    assert latch.on_frame(True, t + 0.1) is None
    assert latch.on_frame(True, t + 0.2) is True


def test_reset_clears_the_confirm_run_without_driving_an_output():
    """Used on the arm edge, so arming while somebody is already standing in
    the box applies the confirm window rather than firing instantly."""
    latch = PresenceLatch(POLICY)
    latch.on_frame(True, 0.0)
    latch.on_frame(True, 0.1)
    latch.reset()
    assert latch.consecutive_in_box == 0
    assert latch.on_frame(True, 0.2) is None
    assert latch.is_on is False


def test_the_confirm_window_is_reported_so_it_is_visible_in_status():
    latch = PresenceLatch(POLICY)
    assert latch.consecutive_in_box == 0
    latch.on_frame(True, 0.0)
    assert latch.consecutive_in_box == 1


def test_a_policy_that_could_never_confirm_is_rejected():
    with pytest.raises(ValueError):
        PresencePolicy(confirm_frames=0)


def test_a_policy_whose_dead_man_precedes_its_minimum_is_rejected():
    """Otherwise the stuck-detector guard fires before the alarm has been
    audible for its minimum duration, and the siren chirps instead of
    sounding."""
    with pytest.raises(ValueError):
        PresencePolicy(min_alert_s=10.0, max_alert_s=5.0)
