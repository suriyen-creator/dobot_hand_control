"""Webcam window and command line entry point. Simulation is the default."""
import argparse
import json
import multiprocessing
import os
from pathlib import Path
import sys
import time
import urllib.request
import zipfile
from .control import Config, Controller, Observation, zone
from .robot import SimRobot

MODEL_URL = ('https://storage.googleapis.com/mediapipe-models/'
             'hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task')
WINDOW = 'Dobot Hand Control'


def parse_args(argv=None):
    p = argparse.ArgumentParser(description='Webcam hand control for Dobot Magician (simulation by default)')
    p.add_argument('--real', action='store_true', help='Connect the original Magician over USB on Windows')
    p.add_argument('--port', help='COM port, e.g. COM3')
    p.add_argument('--sdk-dir', help='Directory containing vendor DobotDllType.py and Windows x64 DLLs')
    p.add_argument('--camera', type=int, help='Webcam index (default: config, otherwise 0)')
    p.add_argument('--config', default='config.json')
    p.add_argument('--model', help='Path to hand_landmarker.task')
    p.add_argument('--download-model', action='store_true', help='Download official hand model, then exit')
    p.add_argument('--headless-demo', action='store_true', help='Run a simulated pick/place without webcam or SDK')
    args = p.parse_args(argv)
    if args.real and args.headless_demo:
        p.error('--real and --headless-demo cannot be combined')
    return args


def key_name(code):
    special = {2424832:'left',2490368:'up',2555904:'right',2621440:'down',
               2162688:'pgup',2228224:'pgdown',
               65361:'left',65362:'up',65363:'right',65364:'down',
               65365:'pgup',65366:'pgdown',32:'space',27:'q'}
    if code in special:
        return special[code]
    return chr(code).lower() if 0 <= code < 128 else None


def download_model(destination):
    path = Path(destination)
    if path.exists():
        raise ValueError(f'{path} already exists; choose --model with a new path to download again')
    path.parent.mkdir(parents=True,exist_ok=True)
    temp = path.with_suffix('.download')
    try:
        with urllib.request.urlopen(MODEL_URL,timeout=30) as response, temp.open('wb') as out:
            size = 0
            while chunk := response.read(1024*1024):
                size += len(chunk)
                if size > 32*1024*1024:
                    raise ValueError('Unexpected model size')
                out.write(chunk)
        with zipfile.ZipFile(temp) as archive:
            if not {'hand_detector.tflite','hand_landmarks_detector.tflite'} <= set(archive.namelist()):
                raise ValueError('Invalid hand landmarker model bundle')
            if archive.testzip():
                raise ValueError('Model download is corrupt')
        temp.replace(path)
    finally:
        temp.unlink(missing_ok=True)
    print(f'Downloaded {path}')


def run_demo():
    robot = SimRobot()
    c = Controller(robot,Config.demo())
    now = 0.0

    def frames(gesture='point', x=.5, y=.5, count=12):
        nonlocal now
        for _ in range(count):
            now += .05
            c.tick(Observation(now,1,gesture,x,y),now)
            if c.state in ('fault','recovery'):
                raise RuntimeError(c.message)

    frames(count=1)
    c.start(now)
    frames(x=.9,count=12)
    frames(gesture='fist',count=300)
    if not c.holding:
        raise RuntimeError('Simulated pickup did not complete')
    frames(count=12)
    frames(y=.1,count=12)
    frames(gesture='open',count=300)
    if c.holding or c.busy:
        raise RuntimeError('Simulated placement did not complete')
    c.pause('Demo finished')
    print(json.dumps(dict(state=c.state,moves=len(robot.moves),
                          pump_events=robot.pump_events,final_z=robot.pose.z)))


def draw(frame, observation, hands, status, real):
    import cv2
    h,w = frame.shape[:2]
    color = (30,220,70) if status['state'] == 'ready' else (0,180,255)
    cx,cy = int(w*.5),int(h*.5)
    cv2.line(frame,(cx,0),(cx,h),(100,100,100),1)
    cv2.line(frame,(0,cy),(w,cy),(100,100,100),1)
    # Diagonals are the boundaries between the four dominant-axis zones.
    cv2.line(frame,(0,0),(w,h),(100,100,100),1)
    cv2.line(frame,(w,0),(0,h),(100,100,100),1)
    cv2.rectangle(frame,(int(w*.35),int(h*.35)),(int(w*.65),int(h*.65)),color,2)
    for hand in hands:
        for point in hand:
            cv2.circle(frame,(int(point.x*w),int(point.y*h)),2,(220,170,40),-1)
    if observation.hands == 1:
        cv2.circle(frame,(int(observation.x*w),int(observation.y*h)),8,color,2)
    for text,pos in [('FORWARD / AWAY',(cx-100,120)),('BACK / TOWARD',(cx-100,h-70)),
                     ('LEFT',(15,cy)),('RIGHT',(w-90,cy)),('STOP',(cx-25,cy))]:
        cv2.putText(frame,text,pos,cv2.FONT_HERSHEY_SIMPLEX,.5,color,1)
    pose = status['pose']
    load = 'UNKNOWN' if status['holding'] is None else ('HOLDING (commanded)' if status['holding'] else 'EMPTY (commanded)')
    direction = zone(observation.x,observation.y) if observation.gesture == 'point' else 'stop'
    lines = [f'{"REAL ROBOT" if real else "SIMULATION"} | {status["state"].upper()} | {load}',
             f'{observation.gesture} / {direction} | hands={observation.hands} | X={pose.x:.1f} Y={pose.y:.1f} Z={pose.z:.1f}',
             status['message']]
    if status['teaching']:
        cfg = status['config']
        lines += [f'TEACH captures={status["captures"]} | W confirmed={status["workspace"]}',
                  'Arrows=XY PgUp/Down=Z | 1/2=corners T=travel P=pick L=place',
                  f'A=swap X=reverse right Y=reverse forward | right={cfg.right_axis}*{cfg.right_sign} forward={cfg.forward_axis}*{cfg.forward_sign}',
                  'W=confirm clear reachable workspace | V=save']
    else:
        lines += ['S=start C=teach Space=stop E=empty+recover H=holding+recover Q=quit']
    # Separate panel keeps instructions readable at every camera resolution.
    panel = frame.copy()
    panel[:] = (25,25,25)
    panel_height = 28 * len(lines) + 12
    panel = cv2.resize(panel,(w,panel_height))
    for i,line in enumerate(lines):
        cv2.putText(panel,line,(8,23+i*28),cv2.FONT_HERSHEY_SIMPLEX,.47,(240,240,240),1)
    return cv2.vconcat([panel,frame])


def run_camera(config, path, real):
    import cv2
    from .vision import HandTracker
    from .runtime import Runtime
    tracker = camera = runtime = robot = None
    try:
        # Validate the camera/model before opening a connection to the robot.
        tracker = HandTracker(config.model)
        camera = cv2.VideoCapture(config.camera, cv2.CAP_DSHOW) if os.name == 'nt' else cv2.VideoCapture(config.camera)
        if not camera.isOpened():
            raise ValueError(f'Cannot open webcam {config.camera}; try --camera 1')
        camera.set(cv2.CAP_PROP_FRAME_WIDTH,960)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
        camera.set(cv2.CAP_PROP_BUFFERSIZE,1)
        if real:
            from .sdk import SdkRobot
            robot = SdkRobot(config.sdk_dir,config.port)
        else:
            robot = SimRobot()
        runtime = Runtime(robot,config,path)
        runtime.start_thread()
        cv2.namedWindow(WINDOW,cv2.WINDOW_NORMAL)
        while True:
            # Timestamp before reading: a stalled capture must never look fresh.
            captured = time.monotonic()
            ok,frame = camera.read()
            if not ok:
                runtime.command('space')
                raise ValueError('Webcam stopped delivering frames')
            frame = cv2.flip(frame,1)
            observation,hands = tracker.detect(frame,captured)
            runtime.observation = observation
            cv2.imshow(WINDOW,draw(frame,observation,hands,runtime.status,real))
            key = key_name(cv2.waitKeyEx(1))
            if key == 'q' or cv2.getWindowProperty(WINDOW,cv2.WND_PROP_VISIBLE) < 1:
                break
            if key:
                runtime.command(key)
            if runtime.status['state'] == 'fault' or not runtime.thread.is_alive():
                raise RuntimeError(runtime.status['message'])
    finally:
        try:
            if runtime and runtime.thread:
                runtime.close()
            elif robot:
                robot.close()
        finally:
            if camera:
                camera.release()
            if tracker:
                tracker.close()
            cv2.destroyAllWindows()


def main(argv=None):
    args = parse_args(argv)
    try:
        if args.headless_demo:
            run_demo()
            return 0
        path = Path(args.config)
        config = Config.load(path) if path.exists() else (Config() if args.real else Config.demo())
        for key in ('port','sdk_dir','camera','model'):
            value = getattr(args,key)
            if value is not None:
                setattr(config,key,value)
        config.validate()
        if args.download_model:
            download_model(config.model)
            return 0
        if args.real and os.name != 'nt':
            raise ValueError('--real requires native Windows x64, not Linux/WSL')
        if not args.real and config.calibration_source == 'real':
            config = Config.demo()
            # Keep camera/model CLI selections without importing real calibration into simulation.
            config.camera = args.camera if args.camera is not None else Config.load(path).camera
            config.model = args.model or Config.load(path).model
        run_camera(config,path,args.real)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f'Error: {exc}',file=sys.stderr)
        return 1
