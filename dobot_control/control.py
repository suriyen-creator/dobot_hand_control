"""Hardware-independent configuration, gesture zones and motion state machine."""
from dataclasses import asdict, dataclass, replace
import json
import math
import time
from pathlib import Path


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    r: float = 0.0

    def distance(self, other):
        return math.dist((self.x, self.y, self.z), (other.x, other.y, other.z))


@dataclass
class Config:
    camera: int = 0
    port: str = ''
    sdk_dir: str = 'sdk'
    model: str = 'models/hand_landmarker.task'
    calibrated: bool = False
    calibration_source: str = 'none'
    x_min: float = 180.0
    x_max: float = 220.0
    y_min: float = -20.0
    y_max: float = 20.0
    pick_z: float = 20.0
    place_z: float = 20.0
    travel_z: float = 50.0
    right_axis: str = 'y'
    right_sign: int = 1
    forward_axis: str = 'x'
    forward_sign: int = 1
    speed: float = 10.0
    teach_speed: float = 5.0
    step_mm: float = 1.0
    gesture_seconds: float = 0.5
    pump_seconds: float = 0.5
    stale_seconds: float = 0.3
    motion_timeout: float = 10.0

    @classmethod
    def demo(cls):
        return cls(calibrated=True, calibration_source='sim')

    def validate(self):
        numeric = ('x_min', 'x_max', 'y_min', 'y_max', 'pick_z', 'place_z',
                   'travel_z', 'speed', 'teach_speed', 'step_mm',
                   'gesture_seconds', 'pump_seconds', 'stale_seconds', 'motion_timeout')
        if any(isinstance(getattr(self, k), bool) or
               not isinstance(getattr(self, k), (int, float)) or
               not math.isfinite(getattr(self, k)) for k in numeric):
            raise ValueError('Configuration numbers must be finite')
        if self.x_min >= self.x_max or self.y_min >= self.y_max:
            raise ValueError('XY minimum must be smaller than maximum')
        if max(self.pick_z, self.place_z) >= self.travel_z:
            raise ValueError('Travel height must be above pick and place heights')
        if not 0 < self.speed <= 50 or not 0 < self.teach_speed <= 5:
            raise ValueError('Speed must be 0..50 mm/s; teaching speed 0..5 mm/s')
        if not 0 < self.step_mm <= 1:
            raise ValueError('Movement step must be at most 1 mm')
        if any(getattr(self, k) <= 0 for k in
               ('gesture_seconds', 'pump_seconds', 'stale_seconds', 'motion_timeout')):
            raise ValueError('Timers must be positive')
        if self.stale_seconds > .3:
            raise ValueError('Frame watchdog must be at most 0.3 seconds')
        if {self.right_axis, self.forward_axis} != {'x', 'y'}:
            raise ValueError('Screen axes must map to different XY axes')
        if self.right_sign not in (-1, 1) or self.forward_sign not in (-1, 1):
            raise ValueError('Axis signs must be -1 or 1')
        if type(self.calibrated) is not bool or type(self.camera) is not int or self.camera < 0:
            raise ValueError('Invalid calibration flag or camera index')
        for name in ('port', 'sdk_dir', 'model'):
            if not isinstance(getattr(self, name), str):
                raise ValueError(f'{name} must be a string')
        return self

    def contains(self, pose):
        return (all(math.isfinite(v) for v in (pose.x, pose.y, pose.z, pose.r)) and
                self.x_min <= pose.x <= self.x_max and
                self.y_min <= pose.y <= self.y_max and
                min(self.pick_z, self.place_z) <= pose.z <= self.travel_z)

    @classmethod
    def load(cls, path):
        return cls(**json.loads(Path(path).read_text(encoding='utf-8'))).validate()

    def save(self, path):
        self.validate()
        path = Path(path)
        temp = path.with_suffix(path.suffix + '.tmp')
        temp.write_text(json.dumps(asdict(self), indent=2) + '\n', encoding='utf-8')
        temp.replace(path)


def zone(x, y):
    if not all(math.isfinite(v) and 0 <= v <= 1 for v in (x, y)):
        return 'stop'
    dx, dy = x - .5, y - .5
    if max(abs(dx), abs(dy)) <= .150000001 or abs(abs(dx) - abs(dy)) < 1e-8:
        return 'stop'
    if abs(dx) > abs(dy):
        return 'right' if dx > 0 else 'left'
    return 'back' if dy > 0 else 'forward'


@dataclass(frozen=True)
class Observation:
    timestamp: float
    hands: int
    gesture: str = 'unknown'
    x: float = .5
    y: float = .5


class Controller:
    def __init__(self, robot, config):
        self.robot = robot
        self.config = config.validate()
        self.state = 'paused'
        self.message = 'Paused - teach positions before starting'
        self.holding = None if robot.real else False
        self.pose = robot.read_pose()
        self.observation = None
        self.pending = None
        self.pending_since = 0.0
        self.phase = None
        self.action = None
        self.phase_since = 0.0
        self.armed = True
        self.gesture = 'unknown'
        self.gesture_since = 0.0
        self.last_tick = None

    @property
    def busy(self):
        return self.state in ('busy', 'recovering')

    def _reset_gesture(self, now=0.0):
        self.gesture = 'unknown'
        self.gesture_since = now

    def start(self, now):
        if self.robot.real and self.config.calibration_source != 'real':
            raise ValueError('Simulator calibration cannot arm real hardware; teach again')
        if self.state in ('recovery', 'fault', 'busy', 'recovering'):
            raise ValueError('Recover interrupted operation before starting')
        if not self.config.calibrated:
            raise ValueError('Teach and save all positions first')
        if self.holding is None:
            raise ValueError('Confirm EMPTY or HOLDING in teaching mode first')
        o = self.observation
        if not o or now - o.timestamp > self.config.stale_seconds or o.hands != 1:
            raise ValueError('Show one hand to start')
        if o.gesture != 'point' or zone(o.x, o.y) != 'stop':
            raise ValueError('Point your index finger in the center to start')
        self.pose = self.robot.read_pose()
        if self.pending or not self.config.contains(self.pose) or abs(self.pose.z - self.config.travel_z) > .2:
            raise ValueError('Move inside the taught area at travel height first')
        self.state, self.message = 'ready', 'Ready'
        self.armed = True
        self._reset_gesture(now)

    def pause(self, reason):
        interrupted = self.busy or self.state == 'recovery'
        self.state = 'recovery' if interrupted else 'paused'
        self.phase = self.action = None
        self._reset_gesture()
        self.message = reason
        try:
            self.robot.stop()
        except Exception as exc:
            self.state = 'fault'
            self.holding = None
            self.message = f'STOP NOT CONFIRMED: {exc}'
        self.pending = None

    def _poll(self, now):
        self.pose = self.robot.read_pose()
        if not all(math.isfinite(v) for v in (self.pose.x, self.pose.y, self.pose.z, self.pose.r)):
            raise RuntimeError('Invalid robot pose')
        if self.pending:
            if now - self.pending_since > self.config.motion_timeout:
                raise RuntimeError('Movement timed out')
            if self.robot.is_idle() and self.pose.distance(self.pending) <= .2:
                self.pending = None

    def _towards(self, target, speed, now, dt):
        if self.pending:
            return False
        distance = self.pose.distance(target)
        if distance <= .05:
            return True
        step = min(self.config.step_mm, speed * dt, distance)
        if step <= 0:
            return False
        ratio = step / distance
        goal = Pose(*(a + (b-a)*ratio for a,b in zip(
            (self.pose.x,self.pose.y,self.pose.z), (target.x,target.y,target.z))), self.pose.r)
        if not self.config.contains(goal):
            raise ValueError('Target outside taught bounds')
        self.robot.move(goal, speed, deadline=self._deadline())
        self.pending, self.pending_since = goal, now
        return False

    def _navigate(self, direction, now, dt):
        if direction == 'stop':
            if self.pending:
                self.robot.stop()
                self.pending = None
            return
        if self.pending:
            return
        axis = self.config.right_axis if direction in ('left','right') else self.config.forward_axis
        sign = self.config.right_sign if direction in ('left','right') else self.config.forward_sign
        if direction in ('left','back'):
            sign *= -1
        value = getattr(self.pose, axis) + sign * min(self.config.step_mm, self.config.speed * dt)
        value = max(getattr(self.config, axis+'_min'), min(getattr(self.config, axis+'_max'), value))
        self._towards(replace(self.pose, **{axis:value}), self.config.speed, now, dt)

    def _deadline(self):
        if self.robot.real and self.state in ('ready', 'busy') and self.observation:
            return self.observation.timestamp + self.config.stale_seconds
        return None

    def _sequence(self, now, dt):
        if self.state == 'recovering':
            if self._towards(replace(self.pose, z=self.config.travel_z), self.config.teach_speed, now, dt):
                self.state, self.message = 'paused', 'Recovered - point at center and press S'
            return
        if self.phase == 'down':
            height = self.config.pick_z if self.action == 'pick' else self.config.place_z
            if self._towards(replace(self.pose, z=height), self.config.teach_speed, now, dt):
                # A timeout leaves the load status unknown; never infer suction success.
                self.holding = None
                self.robot.set_suction(self.action == 'pick', deadline=self._deadline())
                self.holding = self.action == 'pick'
                self.phase, self.phase_since = 'pump', now
        elif self.phase == 'pump':
            if now - self.phase_since >= self.config.pump_seconds:
                self.phase = 'up'
        elif self.phase == 'up':
            if self._towards(replace(self.pose, z=self.config.travel_z), self.config.teach_speed, now, dt):
                self.state, self.message = 'ready', 'Point for 0.5s to rearm'
                self.phase = self.action = None
                self.armed = False
                self._reset_gesture(now)

    def tick(self, observation, now):
        wall_start = time.monotonic()
        self.observation = observation
        dt = .05 if self.last_tick is None else max(0.0, min(.1, now-self.last_tick))
        self.last_tick = now
        if self.state == 'fault':
            return
        try:
            # Check vision before doing any robot I/O or advancing a sequence.
            if self.state in ('ready','busy'):
                if (not observation or now - observation.timestamp > self.config.stale_seconds or
                        observation.timestamp > now or observation.hands != 1):
                    self.pause('Hand lost, multiple hands or stale camera - press S to restart')
                    return
            self._poll(now)
            # Native I/O consumes real time even when the caller uses a simulation clock.
            now += time.monotonic() - wall_start
            if self.state in ('ready', 'busy') and now - observation.timestamp > self.config.stale_seconds:
                self.pause('Camera frame expired during robot I/O - press S to restart')
                return
            if self.state == 'ready' and (not self.config.contains(self.pose) or
                                         abs(self.pose.z - self.config.travel_z) > .2):
                raise ValueError('Robot left the taught travel plane; recovery required')
            if self.busy:
                self._sequence(now, dt)
            elif self.state == 'ready':
                o = observation
                if o.gesture != self.gesture:
                    self.gesture, self.gesture_since = o.gesture, now
                stable = now - self.gesture_since >= self.config.gesture_seconds
                if o.gesture == 'point':
                    if stable:
                        self.armed = True
                    self._navigate(zone(o.x, o.y), now, dt)
                else:
                    self._navigate('stop', now, dt)
                    if self.armed and stable:
                        action = ('pick' if o.gesture == 'fist' and self.holding is False else
                                  'place' if o.gesture == 'open' and self.holding is True else None)
                        if action:
                            self.state, self.action, self.phase = 'busy', action, 'down'
                            self.message = action.upper()
                            self.armed = False
        except Exception as exc:
            self.pause(str(exc))
            if self.state != 'fault':
                self.state = 'recovery'

    def teach_jog(self, axis, sign):
        if self.state not in ('paused','recovery') or axis not in ('x','y','z') or sign not in (-1,1):
            raise ValueError('Pause before teaching')
        if self.pending:
            return
        self.pose = self.robot.read_pose()
        target = replace(self.pose, **{axis:getattr(self.pose, axis)+sign})
        # During initial teaching bounds are unknown: one deliberate 1 mm keypress only.
        self.robot.move(target, self.config.teach_speed)
        self.pending, self.pending_since = target, self.last_tick or 0.0

    def recover(self, holding):
        if self.robot.real and self.config.calibration_source != 'real':
            raise ValueError('Teach real hardware before recovery; simulator calibration is invalid')
        if self.state not in ('paused','recovery') or self.pending:
            raise ValueError('Stop motion before recovery')
        self.pose = self.robot.read_pose()
        if not self.config.calibrated or not self.config.contains(self.pose):
            raise ValueError('Teach bounds and jog inside them before recovery')
        # Operator confirms both the attached object and pump; apply the explicit choice.
        self.holding = None
        self.robot.set_suction(holding)
        self.holding = holding
        self.state, self.message = 'recovering', 'Recovering vertically to travel height'
