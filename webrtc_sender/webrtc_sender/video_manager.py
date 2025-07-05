# handle local video devices
import fractions
import glob

from aiortc import VideoStreamTrack

from av import VideoFrame

import cv2

from linuxpy.video.device import BooleanControl, BufferType, \
    Device, IntegerControl, MenuControl

from .model.video_device import VideoDevice


class OpencvVideoTrack(VideoStreamTrack):
    def __init__(
            self, video_dev: str, width: int, height: int,
            max_fps: int = 30):
        super().__init__()
        self.frame_count = 0
        self.original_width = width
        self.original_height = height
        self.max_fps = max_fps

        self.cap = cv2.VideoCapture(video_dev)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, max_fps)

    async def recv(self):
        self.frame_count += 1
        ret, frame = self.cap.read()
        if not ret:
            print('Failed to read frame from camera.')
            return None
        video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = self.frame_count
        video_frame.time_base = fractions.Fraction(1, 30)
        return video_frame


def _parse_control(ctrl):
    # convert control information to JSON-serializable dicts
    if isinstance(ctrl, IntegerControl):
        return {
            'type': 'int',
            'writable': ctrl.is_writeable,
            'name': ctrl.name,
            'min': ctrl.minimum,
            'max': ctrl.maximum,
            'step': ctrl.step,
            'value': ctrl.value
        }
    elif isinstance(ctrl, BooleanControl):
        return {
            'type': 'bool',
            'writable': ctrl.is_writeable,
            'name': ctrl.name,
            'value': ctrl.value
        }
    elif isinstance(ctrl, MenuControl):
        return {
            'type': 'menu',
            'writable': ctrl.is_writeable,
            'name': ctrl.name,
            'value': ctrl.value,
            'options': list(ctrl.items())
        }


def read_cv2_data(video_dev):
    common_resolutions = [
        (160, 120),    # 4:3
        (320, 240),    # 4:3
        (640, 480),    # VGA, 4:3
        (800, 600),    # SVGA, 4:3
        (1024, 768),   # XGA, 4:3
        (1280, 720),   # HD, 16:9
        (1280, 1024),  # SXGA, 5:4
        (1920, 1080),  # Full HD, 16:9
        (2560, 1440),  # QHD, 16:9
        (3840, 2160),  # 4K UHD, 16:9
    ]

    cap = cv2.VideoCapture(video_dev)

    if not cap.isOpened():
        # We could not open the camera device, so lets ignore it!")
        return None

    max_fps = cap.get(cv2.CAP_PROP_FPS)

    original_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    original_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    supported_resolutions = [(original_width, original_height)]

    for width, height in common_resolutions:
        if width == original_width and height == original_height:
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        # Read back the resolution to see if it was accepted
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # Some drivers might round the resolution, so we allow small tolerances
        if abs(actual_width - width) < 3 and abs(actual_height - height) < 3:
            supported_resolutions.append([width, height])

    # Restore the original resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, original_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, original_height)

    cap.release()
    return max_fps, supported_resolutions


class VideoManager:
    devices = []
    tracks = []

    def __init__(self):
        self.devices = self.get_available_cams()

    def get_available_cams(self):
        data = []
        for video_dev in glob.glob('/dev/video*'):
            video_id = int(video_dev[10:])
            dev = Device.from_id(video_id)
            dev.open()
            driver = dev.info.driver
            name = dev.info.card
            try:
                formats = [
                    x.description for x in dev.info.formats
                    if x.type != BufferType.META_CAPTURE  # filter out meta
                ]
            except UnboundLocalError:
                formats = None
            if not formats:
                # no supported types, probably just a
                # META-CAPUTRE device we can skip
                dev.close()
                continue

            ctrls = []
            try:
                for ctrl in dev.controls.values():
                    _ctrl = _parse_control(ctrl)
                    if _ctrl:
                        ctrls.append(_ctrl)
            except ValueError:
                pass

            dev.close()
            cv2_data = read_cv2_data(video_dev)
            if not cv2_data:
                # No cv2 data, we were not able to open the device
                # ignore unusable devices.
                continue
            max_fps, resolutions = cv2_data
            vd = VideoDevice(
                video_dev=video_dev,
                driver=driver,
                name=name,
                formats=formats,
                controls=ctrls,
                max_fps=max_fps,
                resolutions=resolutions)
            data.append(vd)

        return data


if __name__ == '__main__':
    mgr = VideoManager()
