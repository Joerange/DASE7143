"""Small asynchronous keyboard controller; final velocity remains in guard."""
from .core import fresh
from .timing import SERVICE_TIMEOUT,STOP_WAIT_TIMEOUT


class WindowControl:
    def __init__(self, send):
        self.send = send
        self.stage = 'idle'
        self.message = 'Monitor: S start | SPACE stop | Q quit'
        self.pending = None
        self.late_arm = None
        self.deadline = 0.
        self.retry_at = 0.
        self.still_since = None
        self.closing = False
        self.exit_ready = False
        self.safety = {}
        self.mission = {}
        self.odom = {}

    def start(self, now):
        if self.stage != 'idle' or self.closing or self.mission.get('active'):
            return
        self.pending = self.send('arm', True)
        if self.pending is None:
            self.message = 'Waiting for safety service; press S when ready'
            return
        self.stage = 'arming'
        self.deadline = now + SERVICE_TIMEOUT
        self.message = 'Requesting motion permission...'

    def stop(self, now, closing=False):
        self.closing = self.closing or closing
        if self.stage == 'stopping':
            return
        if self.stage == 'arming':
            self.late_arm = self.pending
        self.pending = None
        self.stage = 'stopping'
        self.deadline = now + STOP_WAIT_TIMEOUT
        self.still_since = None
        self.message = 'Stopping: disabling motion and stopping task...'
        self._send_stop(now)

    def _send_stop(self, now):
        # Both calls are asynchronous: video/model readiness never gates stop.
        self.send('arm', False)
        self.send('stop')
        self.retry_at = now + .5

    def key(self, key, now):
        if key in (ord('s'), ord('S'), 13):
            self.start(now)
        elif key in (32, ord('x'), ord('X')):
            self.stop(now)
        elif key in (ord('q'), ord('Q'), 27):
            self.stop(now, closing=True)

    def tick(self, now, ros_now):
        if self.late_arm is not None and self.late_arm.done():
            self.send('arm', False)
            self.late_arm = None
        if self.stage in ('arming', 'starting'):
            if self.pending.done():
                try:
                    response = self.pending.result()
                    success, message = response.success, response.message
                except Exception as error:
                    success, message = False, type(error).__name__
                if not success:
                    stage = self.stage
                    self.send('arm', False)
                    self.stage = 'idle'
                    self.message = ('Permission denied: ' if stage == 'arming' else 'Start failed: ') + message
                elif self.stage == 'arming':
                    self.pending = None
                    self.stage = 'waiting_ready'
                    self.deadline = now + SERVICE_TIMEOUT
                    self.message = 'Motion granted; waiting for task readiness...'
                else:
                    self.stage = 'idle'
                    self.message = 'Task started | SPACE stop | Q quit'
            elif now > self.deadline:
                self.stop(now)
                self.message = 'Start timed out; disabling motion and stopping task'
        if self.stage == 'waiting_ready':
            # A successful SetBool response does not prove that the mission's
            # independent subscription has received the new authorization yet.
            if (fresh(self.safety,now,ros_now) and self.safety.get('authorized') and self.safety.get('ready')
                    and fresh(self.mission,now,ros_now) and self.mission.get('start_ready')):
                self.pending = self.send('start')
                if self.pending is None:
                    self.send('arm',False);self.stage='idle'
                    self.message='Task service unavailable; motion disabled'
                else:
                    self.stage='starting';self.deadline=now+SERVICE_TIMEOUT;self.message='Starting task...'
            elif now>self.deadline:
                reason=self.mission.get('start_block_reason','task readiness unavailable')
                self.stop(now)
                self.message='Start blocked: '+reason
        if self.stage == 'stopping':
            if now >= self.retry_at:
                self._send_stop(now)
            safe = (fresh(self.safety, now, ros_now) and not self.safety.get('authorized', True)
                    and self.safety.get('output_zero') and fresh(self.mission, now, ros_now)
                    and not self.mission.get('active', True) and self.late_arm is None)
            preview = self.safety.get('output_topic') == '/rgbd/cmd_vel_preview'
            measured_still = (fresh(self.odom, now, ros_now)
                              and abs(self.odom.get('v', float('inf'))) < .015
                              and abs(self.odom.get('w', float('inf'))) < .025
                              and self.safety.get('feedback_live')
                              and self.safety.get('feedback_stopped'))
            if safe and (preview or measured_still):
                if self.still_since is None:
                    self.still_since = now
                if now - self.still_since >= .7:
                    self.stage = 'idle'
                    self.message = 'Preview stopped' if preview else 'Stopped; motion disabled'
                    self.exit_ready = self.closing
            else:
                self.still_since = None
            if self.stage == 'stopping' and now > self.deadline:
                self.message = 'Stop result UNKNOWN: no fresh standstill confirmation'
                # Closing must still reap owned processes. Never report this as stopped.
                self.exit_ready = self.closing
