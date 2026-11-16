"""Runtime motion permission. This latch is never restored from parameters/disk.

The console lease and passive chassis health only establish permission to begin
an explicitly started mission; neither contains nor generates a velocity.
"""
from dataclasses import dataclass
import math
import uuid
from .core import Sample, fresh


@dataclass(frozen=True)
class AuthorityConfig:
    capable: bool = False
    heartbeat_timeout: float = .5
    chassis_timeout: float = .5

    def __post_init__(self):
        for value in (self.heartbeat_timeout, self.chassis_timeout):
            if not math.isfinite(value) or not 0 < value <= 2.:
                raise ValueError('Authority freshness limits must be within (0, 2] seconds')


class MotionAuthority:
    def __init__(self, config=None):
        self.config = config or AuthorityConfig()
        self.authorized = False
        self.reason = 'startup_disarmed'
        self.boot_id = uuid.uuid4().hex
        self.revision = 0
        self.control_session = ''
        self.bound_session = ''
        self.retired_sessions = set()
        self.heartbeat = self.chassis = None

    @property
    def session(self):
        return self.boot_id + ':' + str(self.revision) + ':' + self.control_session

    @staticmethod
    def envelope(data):
        return (isinstance(data, dict) and data.get('schema') == 1 and
                isinstance(data.get('session_id'), str) and 0 < len(data['session_id']) <= 128 and
                isinstance(data.get('stamp'), (float, int)) and not isinstance(data['stamp'], bool) and
                math.isfinite(data['stamp']))

    def revoke(self, reason):
        was_authorized = self.authorized
        if self.authorized or self.reason != reason:
            self.revision += 1
        self.authorized = False
        self.bound_session = ''
        self.reason = reason
        return was_authorized

    def update_heartbeat(self, data, now):
        if not self.envelope(data):
            self.heartbeat = None
            self.revoke('control_heartbeat_invalid')
            return False
        session = data['session_id']
        if session in self.retired_sessions:
            return False
        if session != self.control_session:
            if self.control_session:
                self.retired_sessions.add(self.control_session)
                self.revoke('control_session_changed')
            self.control_session = session
            self.chassis = None
        elif self.heartbeat is not None and data['stamp'] <= self.heartbeat.stamp:
            return False  # Replays cannot refresh the monotonic receipt clock.
        self.heartbeat = Sample(float(data['stamp']), now, dict(data))
        return True

    def update_chassis(self, data, now):
        if (not self.envelope(data) or
                any(type(data.get(k)) is not bool for k in ('communication_ok', 'driver_running', 'can_up')) or
                not isinstance(data.get('can_interface'), str) or not data['can_interface']):
            self.chassis = None
            self.revoke('chassis_health_invalid')
            return False
        if data['session_id'] in self.retired_sessions:
            return False
        if (self.chassis is not None and data['session_id'] == self.chassis.value['session_id']
                and data['stamp'] <= self.chassis.stamp):
            return False
        self.chassis = Sample(float(data['stamp']), now, dict(data))
        if not all(data[k] for k in ('communication_ok', 'driver_running', 'can_up')):
            reason = ('can_interface_down' if not data['can_up'] else
                      'chassis_driver_not_running' if not data['driver_running'] else
                      'chassis_communication_lost')
            self.revoke(reason)
        return True

    def requirements(self, now, ros_now, sensor_ready, sensor_reason,
                     mission_fresh, navigation_ready):
        if not self.config.capable:
            return False, 'runtime_control_disabled'
        if not fresh(self.heartbeat, now, ros_now, self.config.heartbeat_timeout):
            return False, 'control_heartbeat_stale'
        if not fresh(self.chassis, now, ros_now, self.config.chassis_timeout):
            return False, 'chassis_health_stale'
        c = self.chassis.value
        if c['session_id'] != self.control_session:
            return False, 'chassis_session_mismatch'
        if not c['can_up']:
            return False, 'can_interface_down'
        if not c['driver_running']:
            return False, 'chassis_driver_not_running'
        if not c['communication_ok']:
            return False, 'chassis_communication_lost'
        if not mission_fresh:
            return False, 'mission_state_stale'
        if not navigation_ready:
            return False, 'nav2_not_ready'
        if not sensor_ready:
            return False, 'sensor_health:' + sensor_reason
        return True, 'ready_to_authorize'

    def request(self, enabled, now, ros_now, sensor_ready, sensor_reason,
                mission_fresh, navigation_ready, source_stopped, actual_stopped):
        if not enabled:
            self.revoke('operator_disarmed')
            return True, self.reason
        ready, reason = self.requirements(now, ros_now, sensor_ready, sensor_reason,
                                          mission_fresh, navigation_ready)
        if not ready:
            self.revoke(reason)
            return False, reason
        if self.authorized and self.bound_session == self.control_session:
            return True, 'already_authorized'
        if not source_stopped:
            self.revoke('mission_must_be_stopped_before_authorization')
            return False, self.reason
        if not actual_stopped:
            self.revoke('actual_robot_not_stopped')
            return False, self.reason
        self.revision += 1
        self.authorized = True
        self.bound_session = self.control_session
        self.reason = 'operator_authorized'
        return True, self.reason

    def monitor(self, now, ros_now, sensor_ready, sensor_reason, mission_fresh, navigation_ready):
        ready, reason = self.requirements(now, ros_now, sensor_ready, sensor_reason,
                                          mission_fresh, navigation_ready)
        if self.authorized and (not ready or self.bound_session != self.control_session):
            self.revoke(reason if not ready else 'control_session_changed')
            return True
        return False

    def status(self):
        return dict(authorized=self.authorized, authorization_reason=self.reason,
                    authority_session=self.session, control_session=self.control_session,
                    runtime_control_capable=self.config.capable)
