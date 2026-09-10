from camera.recorder import SimulationRecorder


async def test_preflight_check_is_a_noop_that_says_so(capsys):
    # It must not pass silently: a preflight log that reads as if a real
    # camera had been checked would be actively misleading.
    recorder = SimulationRecorder()
    await recorder.preflight_check()
    assert "SimulationRecorder" in capsys.readouterr().out
    assert recorder.is_recording is False


async def test_starts_not_recording():
    recorder = SimulationRecorder()
    assert recorder.is_recording is False


async def test_start_recording_sets_state():
    recorder = SimulationRecorder()
    await recorder.start_recording()
    assert recorder.is_recording is True


async def test_stop_recording_clears_state():
    recorder = SimulationRecorder()
    await recorder.start_recording()
    await recorder.stop_recording()
    assert recorder.is_recording is False


async def test_double_start_does_not_raise():
    # A bookkeeping error here must never be able to abort a healthy
    # mission -- see the project plan's note on the original Camera
    # class raising RuntimeError on a duplicate start.
    recorder = SimulationRecorder()
    await recorder.start_recording()
    await recorder.start_recording()
    assert recorder.is_recording is True


async def test_stop_without_start_does_not_raise():
    recorder = SimulationRecorder()
    await recorder.stop_recording()
    assert recorder.is_recording is False


async def test_stop_is_idempotent():
    recorder = SimulationRecorder()
    await recorder.start_recording()
    await recorder.stop_recording()
    await recorder.stop_recording()
    assert recorder.is_recording is False
