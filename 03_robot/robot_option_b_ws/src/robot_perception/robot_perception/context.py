"""Acquisition context survives ordinary map updates, never SLAM frame resets."""
from dataclasses import dataclass, field
import math


def can_request(state, reconfirm_required):
    return state == 'SEARCH_CONTINUOUS' or (state == 'REOBSERVE_RECOVERY' and reconfirm_required is True)


@dataclass
class ObservationContext:
    generation: str = ''
    geometry_epoch: int = 0
    state: str = 'INIT'
    reconfirm_required: bool = False
    since_stamp: float = float('inf')
    mission_stamp: float = -float('inf')
    received: float = -float('inf')
    retired: set = field(default_factory=set)

    @property
    def key(self):
        return self.generation, self.geometry_epoch

    def update(self, data, ros_now, received):
        """Accept current mission heartbeats, rejecting retired/reordered contexts."""
        if not isinstance(data, dict) or data.get('schema') != 1:
            return False
        generation, epoch = data.get('task_generation'), data.get('geometry_epoch', 0)
        stamp = data.get('stamp')
        if (not isinstance(generation, str) or not generation or type(epoch) is not int or epoch < 0 or
                not isinstance(stamp, (float, int)) or isinstance(stamp, bool) or not math.isfinite(stamp) or
                not -.05 <= ros_now-stamp <= 1. or not isinstance(data.get('state'), str) or
                type(data.get('reconfirm_required', False)) is not bool or generation in self.retired):
            return False
        if stamp < self.mission_stamp or (self.generation == generation and epoch < self.geometry_epoch):
            return False
        key = generation, epoch
        if key != self.key:
            if self.generation and generation != self.generation:
                self.retired.add(self.generation)
            self.since_stamp = float(stamp)
        self.generation, self.geometry_epoch = key
        self.state, self.reconfirm_required = data['state'], data.get('reconfirm_required', False)
        self.mission_stamp, self.received = float(stamp), received
        return True

    def matches(self, data):
        return (data.get('task_generation') == self.generation and
                data.get('geometry_epoch', 0) == self.geometry_epoch)

    def accepts_capture(self, stamp):
        return bool(self.generation) and math.isfinite(stamp) and stamp >= self.since_stamp

    def permits_event(self, ros_now, now):
        return (0 <= now-self.received <= 1. and -.05 <= ros_now-self.mission_stamp <= 1. and
                can_request(self.state, self.reconfirm_required))
