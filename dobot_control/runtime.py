"""Own all robot I/O on one thread, independent of webcam/inference latency."""
from dataclasses import replace
import queue
import threading
import time
from .control import Controller
from .teaching import Teaching


class Runtime:
    def __init__(self, robot, config, config_path):
        self.controller = Controller(robot,config)
        self.config_path = config_path
        self.observation = None
        self.status = {}
        self.commands = queue.Queue(maxsize=8)
        self.stop_requested = threading.Event()
        self.done = threading.Event()
        self.thread = None
        self.teaching = None
        self.workspace_confirmed = False
        self._publish()

    def start_thread(self):
        self.thread = threading.Thread(target=self._run, name='robot-control', daemon=True)
        self.thread.start()

    def command(self, key):
        if key == 'space':
            self.stop_requested.set()
            return
        try:
            self.commands.put_nowait((key,time.monotonic()))
        except queue.Full:
            self.stop_requested.set()

    def _clear_commands(self):
        while True:
            try:
                self.commands.get_nowait()
            except queue.Empty:
                return

    def _publish(self):
        c = self.controller
        self.status = dict(state=c.state, message=c.message, pose=c.pose,
                           holding=c.holding, teaching=self.teaching is not None,
                           captures=''.join(sorted(self.teaching.captures)) if self.teaching else '',
                           workspace=self.workspace_confirmed, config=replace(c.config))

    def handle(self, key, now):
        c = self.controller
        if key == 's':
            if self.teaching:
                raise ValueError('Save teaching with V before starting')
            c.start(now)
        elif key == 'c':
            c.pause('Teaching: arrows XY, PgUp/PgDn Z; captures 1/2/T/P/L, W confirm, V save')
            source = 'real' if c.robot.real else 'sim'
            self.teaching = Teaching(replace(c.config,calibration_source=source))
            c.config = self.teaching.config
            self.workspace_confirmed = False
        elif key in ('e','h'):
            if self.teaching:
                raise ValueError('Save teaching first, then E=empty or H=holding to recover')
            c.recover(key == 'h')
        elif self.teaching:
            if key in ('left','right','up','down','pgup','pgdown'):
                cfg = self.teaching.config
                axis,sign = {
                    'left':(cfg.right_axis,-cfg.right_sign),
                    'right':(cfg.right_axis,cfg.right_sign),
                    'up':(cfg.forward_axis,cfg.forward_sign),
                    'down':(cfg.forward_axis,-cfg.forward_sign),
                    'pgup':('z',1),'pgdown':('z',-1)}[key]
                c.teach_jog(axis,sign)
            elif key in ('1','2','t','p','l'):
                if c.pending:
                    raise ValueError('Wait for movement to finish before capturing')
                self.teaching.capture(key,c.robot.read_pose())
                self.workspace_confirmed = False
                c.message = f'Captured {key.upper()}'
            elif key in ('a','x','y'):
                if c.pending:
                    raise ValueError('Wait for movement before changing axes')
                if key == 'a':
                    self.teaching.swap_axes()
                else:
                    self.teaching.reverse('right' if key == 'x' else 'forward')
                c.config = self.teaching.config
                c.message = (f'Right={c.config.right_axis}*{c.config.right_sign}; '
                             f'Forward={c.config.forward_axis}*{c.config.forward_sign}')
            elif key == 'w':
                self.workspace_confirmed = True
                c.message = 'Workspace confirmed; V saves all captures'
            elif key == 'v':
                if c.pending:
                    raise ValueError('Wait for movement before saving')
                cfg = self.teaching.finish(self.workspace_confirmed)
                cfg.save(self.config_path)
                c.config = cfg
                self.teaching = None
                c.message = 'Saved. E=confirm empty / H=confirm holding, then recover to travel Z'
            else:
                raise ValueError('Unknown teaching key')
        else:
            raise ValueError('C=teach, S=start, Space=stop, E/H=recover, Q=quit')

    def _run(self):
        c = self.controller
        try:
            while not self.done.is_set():
                now = time.monotonic()
                c.observation = self.observation
                if self.stop_requested.is_set():
                    self.stop_requested.clear()
                    self._clear_commands()
                    c.pause('Operator stop - press S, or E/H to recover interrupted action')
                else:
                    try:
                        key,stamp = self.commands.get_nowait()
                    except queue.Empty:
                        pass
                    else:
                        if now - stamp <= .3:
                            try:
                                self.handle(key,now)
                            except ValueError as exc:
                                c.message = str(exc)
                c.tick(self.observation,time.monotonic())
                self._publish()
                self.done.wait(.05)
        except Exception as exc:
            c.pause(f'Control error: {exc}')
            self._publish()
        finally:
            c.pause('Exiting - pump state preserved')
            if c.state == 'fault':
                print(c.message,flush=True)
            try:
                c.robot.close()
            except Exception as exc:
                print(f'STOP NOT CONFIRMED on close: {exc}',flush=True)
            self._publish()

    def close(self):
        self.done.set()
        if self.thread:
            self.thread.join(timeout=4)
            if self.thread.is_alive():
                raise RuntimeError('STOP NOT CONFIRMED: robot thread did not exit')
