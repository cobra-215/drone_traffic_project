"""
Drone() is safe to construct without a running PX4 -- mavsdk.System's
constructor only sets attributes; the mavsdk_server subprocess is spawned
inside connect(), which none of these tests call. That lets Drone.disconnect()
be unit-tested directly against a real (never-connected) Drone/System for the
happy path, and against lightweight fakes for the edge cases -- monkeypatching
methods directly onto a real mavsdk.System instance risks tripping its own
__del__ (which also calls _stop_mavsdk_server) with our test stub during
garbage collection, independent of the test under exercise.
"""

from flight.drone import Drone


def test_disconnect_calls_stop_mavsdk_server():
    drone = Drone()

    calls = []
    drone.system._stop_mavsdk_server = lambda: calls.append(True)

    drone.disconnect()

    assert calls == [True]

    # Avoid mavsdk.System.__del__ later invoking our stub during garbage
    # collection of this real System instance.
    del drone.system._stop_mavsdk_server


def test_disconnect_is_a_noop_when_stop_mavsdk_server_missing():
    class FakeSystemWithoutStop:
        pass

    drone = Drone()
    drone.system = FakeSystemWithoutStop()  # simulate a MAVSDK API change

    drone.disconnect()  # must not raise


def test_disconnect_swallows_errors_from_stop_mavsdk_server():
    class FakeSystemThatFails:
        def _stop_mavsdk_server(self):
            raise RuntimeError("subprocess already reaped")

    drone = Drone()
    drone.system = FakeSystemThatFails()

    drone.disconnect()  # must not raise -- cleanup failure is best-effort


def test_disconnect_is_safe_before_connect_was_ever_called():
    # The real regression this guards: calling disconnect() in a finally
    # block even when connect() failed or was never reached must not
    # itself raise (mavsdk.System's own _stop_mavsdk_server is a no-op
    # when _server_process was never set).
    drone = Drone()
    drone.disconnect()
