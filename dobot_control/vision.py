"""Conservative, rotation-independent finger rules over MediaPipe landmarks."""
import math
from pathlib import Path
from .control import Observation


def angle(a, b, c):
    u = tuple(x-y for x,y in zip(a,b))
    v = tuple(x-y for x,y in zip(c,b))
    size = math.sqrt(sum(x*x for x in u) * sum(x*x for x in v))
    if size < 1e-12:
        return 0.0
    return math.degrees(math.acos(max(-1., min(1., sum(x*y for x,y in zip(u,v))/size))))


def classify(points):
    if len(points) != 21 or any(not math.isfinite(v) for p in points for v in p):
        return 'unknown'
    if math.dist(points[0], points[9]) < 1e-6:
        return 'unknown'
    extended, curled = [], []
    for base in (5,9,13,17):
        pip = angle(points[base], points[base+1], points[base+2])
        dip = angle(points[base+1], points[base+2], points[base+3])
        extended.append(pip > 155 and dip > 155 and
                        math.dist(points[base+3],points[0]) > math.dist(points[base+1],points[0]))
        curled.append(pip < 135)
    thumb = (angle(points[1],points[2],points[3]) > 150 and
             angle(points[2],points[3],points[4]) > 150 and
             math.dist(points[4],points[5]) > math.dist(points[3],points[5]))
    if all(extended) and thumb:
        return 'open'
    if all(curled) and not thumb:
        return 'fist'
    if extended[0] and all(curled[1:]) and not thumb:
        return 'point'
    return 'unknown'


def palm_center(points):
    return tuple(sum(points[i][axis] for i in (0,5,9,13,17))/5 for axis in (0,1))


class HandTracker:
    def __init__(self, model):
        import mediapipe as mp
        from mediapipe.tasks import python
        from mediapipe.tasks.python import vision
        if not Path(model).is_file():
            raise ValueError('Hand model missing: run python -m dobot_control --download-model')
        self.mp = mp
        self.last_ms = -1
        self.landmarker = vision.HandLandmarker.create_from_options(
            vision.HandLandmarkerOptions(
                base_options=python.BaseOptions(model_asset_path=str(model)),
                running_mode=vision.RunningMode.VIDEO,
                num_hands=2,
                min_hand_detection_confidence=.7,
                min_hand_presence_confidence=.7,
                min_tracking_confidence=.7))

    def detect(self, bgr, timestamp):
        import cv2
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        stamp = max(self.last_ms + 1, int(timestamp*1000))
        self.last_ms = stamp
        result = self.landmarker.detect_for_video(
            self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb), stamp)
        hands = result.hand_landmarks
        if len(hands) != 1:
            return Observation(timestamp, len(hands)), hands
        raw = [(p.x,p.y,p.z) for p in hands[0]]
        h,w = bgr.shape[:2]
        # Correct normalized image aspect ratio before calculating joint angles.
        scaled = [(x*w/max(w,h),y*h/max(w,h),z*w/max(w,h)) for x,y,z in raw]
        x,y = palm_center(raw)
        return Observation(timestamp,1,classify(scaled),x,y), hands

    def close(self):
        self.landmarker.close()
