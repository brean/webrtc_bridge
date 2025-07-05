from dataclasses import dataclass


@dataclass
class VideoDevice():
    name: str = 'Generic cam'

    # driver e.g. uvcvideo
    driver: str = 'none'

    # OpenCV usable device index for capturing
    video_dev: str = '/dev/video0'

    # -- data from linuxpy --
    # Type (rgb, infrared, ...)
    formats: list = None

    # camera options we can set
    controls: list = None

    # -- data from OpenCV2 --
    # supported resultions
    resolutions: list = None

    max_fps: int = 0
