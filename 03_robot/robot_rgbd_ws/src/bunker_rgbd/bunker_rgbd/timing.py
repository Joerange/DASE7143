"""Runtime timeout limits: three times the original values, per operator request."""
TIMEOUT_SCALE=3.


def scaled(seconds):return round(seconds*TIMEOUT_SCALE,6)


DATA_TIMEOUT=scaled(.3)
OBSERVATION_TIMEOUT=scaled(.3)
COMMAND_TIMEOUT=scaled(.2)
CAN_FEEDBACK_TIMEOUT=scaled(.3)
CAN_RECEIVE_TIMEOUT=scaled(.2)
TRACK_TIMEOUT=scaled(.3)
SERVICE_TIMEOUT=scaled(3.)
STOP_WAIT_TIMEOUT=scaled(5.)
WORKER_EXIT_TIMEOUT=scaled(5.)
MISSION_TIMEOUT=scaled(150.)
SEARCH_TIMEOUT=scaled(65.)
LAUNCH_EXIT_GRACE=scaled(1.)
LAUNCH_SIGNAL_TIMEOUT=scaled(5.)
