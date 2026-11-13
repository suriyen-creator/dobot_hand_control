"""Run the vendor's DobotDllType.py in a disposable, time-bounded process.

Vendor wrappers may retry forever. Never call them from the camera or control
thread. A failed RPC is terminal: a late command must not execute after recovery.
"""
import importlib.util
import multiprocessing
import os
from pathlib import Path
import sys
import time
from .control import Pose


def _sdk_worker(pipe, sdk_dir, port):
    connected = False
    d = api = None
    try:
        directory = Path(sdk_dir).resolve()
        os.chdir(directory)
        sys.path.insert(0, str(directory))
        dll_directory = os.add_dll_directory(str(directory)) if os.name == 'nt' else None
        spec = importlib.util.spec_from_file_location('DobotDllType', directory / 'DobotDllType.py')
        d = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(d)
        api = d.load()
        result = d.ConnectDobot(api, port, 115200)
        if not result or result[0] != 0:
            raise RuntimeError(f'ConnectDobot failed: {result}')
        connected = True
        d.SetCmdTimeout(api, 100)
        d.SetQueuedCmdForceStopExec(api)
        d.SetQueuedCmdClear(api)
        d.SetPTPCommonParams(api, 100, 100, isQueued=0)
        last_index = None
        last_speed = None
        pipe.send((True, None))
        while True:
            command, args = pipe.recv()
            try:
                if command == 'pose':
                    result = tuple(float(v) for v in d.GetPose(api)[:4])
                elif command == 'move':
                    pose, speed, deadline = args
                    if deadline is not None and time.monotonic() >= deadline:
                        raise RuntimeError('Frame expired before SDK movement')
                    if speed != last_speed:
                        d.SetPTPCoordinateParams(api, speed, 20, 5, 20, isQueued=0)
                        last_speed = speed
                    if deadline is not None and time.monotonic() >= deadline:
                        raise RuntimeError('Frame expired while configuring movement')
                    last_index = d.SetPTPCmd(api, d.PTPMode.PTPMOVLXYZMode,
                                            pose.x,pose.y,pose.z,pose.r,isQueued=1)[0]
                    d.SetQueuedCmdStartExec(api)
                    result = None
                elif command == 'idle':
                    result = last_index is None or d.GetQueuedCmdCurrentIndex(api)[0] >= last_index
                elif command == 'stop':
                    d.SetQueuedCmdForceStopExec(api)
                    d.SetQueuedCmdClear(api)
                    last_index = None
                    result = None
                elif command == 'suction':
                    if args[1] is not None and time.monotonic() >= args[1]:
                        raise RuntimeError('Frame expired before pump command')
                    d.SetEndEffectorSuctionCup(api, True, args[0], isQueued=0)
                    status = d.GetEndEffectorSuctionCup(api)
                    if len(status) < 2 or bool(status[1]) != args[0] or not status[0]:
                        raise RuntimeError('Pump command could not be confirmed')
                    result = None
                elif command == 'pump_state':
                    result = bool(d.GetEndEffectorSuctionCup(api)[1])
                elif command == 'close':
                    # Closing preserves the pump. The controller requests a stop first.
                    d.DisconnectDobot(api)
                    connected = False
                    pipe.send((True,None))
                    return
                else:
                    raise ValueError('Unknown SDK operation')
                pipe.send((True,result))
            except Exception as exc:
                pipe.send((False,str(exc)))
    except (EOFError, BrokenPipeError):
        pass
    except Exception as exc:
        try:
            pipe.send((False,str(exc)))
        except (EOFError,BrokenPipeError,OSError):
            pass
    finally:
        if connected:
            try:
                d.SetQueuedCmdForceStopExec(api)
                d.SetQueuedCmdClear(api)
                d.DisconnectDobot(api)
            except Exception:
                pass
        pipe.close()


class SdkRobot:
    real = True

    def __init__(self, sdk_dir, port):
        if not port:
            raise ValueError('Specify a COM port with --port or config.json')
        directory = Path(sdk_dir).resolve()
        if not (directory / 'DobotDllType.py').is_file():
            raise ValueError(f'Missing {directory / "DobotDllType.py"}; see README.md')
        self.timeout = .2
        self.failed = False
        context = multiprocessing.get_context('spawn')
        self.pipe, child = context.Pipe()
        self.process = context.Process(target=_sdk_worker, args=(child,str(directory),port), daemon=True)
        self.process.start()
        child.close()
        try:
            self._receive(10.0)
        except Exception:
            self._terminate()
            raise

    def _terminate(self):
        self.failed = True
        if self.process.is_alive():
            self.process.terminate()
        self.process.join(timeout=1)
        self.pipe.close()

    def _receive(self, timeout):
        try:
            if not self.pipe.poll(timeout):
                raise TimeoutError('SDK response timed out')
            ok, result = self.pipe.recv()
        except (EOFError,OSError,TimeoutError) as exc:
            self._terminate()
            raise RuntimeError(f'STOP NOT CONFIRMED: {exc}; restart and recover manually') from exc
        if not ok:
            # Stop processing immediately; callers cannot resume a possibly failed command.
            self._terminate()
            raise RuntimeError(f'SDK error; STOP NOT CONFIRMED: {result}')
        return result

    def _call(self, operation, *args, deadline=None):
        if self.failed or not self.process.is_alive():
            raise RuntimeError('STOP NOT CONFIRMED: SDK disconnected')
        timeout = self.timeout
        if deadline is not None:
            timeout = min(timeout, deadline - time.monotonic())
            if timeout <= 0:
                raise ValueError('Camera frame expired before SDK command')
        try:
            self.pipe.send((operation,args))
        except (OSError,EOFError) as exc:
            self._terminate()
            raise RuntimeError(f'STOP NOT CONFIRMED: {exc}') from exc
        return self._receive(timeout)

    def read_pose(self):
        return Pose(*self._call('pose'))

    def move(self, target, speed, deadline=None):
        self._call('move',target,speed,deadline,deadline=deadline)

    def is_idle(self):
        return self._call('idle')

    def stop(self):
        self._call('stop')

    def set_suction(self, enabled, deadline=None):
        self._call('suction',enabled,deadline,deadline=deadline)

    def suction_state(self):
        return self._call('pump_state')

    def close(self):
        if not self.failed:
            try:
                self.stop()
                self._call('close')
            finally:
                self._terminate()
