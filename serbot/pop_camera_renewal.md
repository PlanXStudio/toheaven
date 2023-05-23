# ~/pop/__init__.py
```pyton
class NewCamera:
    def __new__(cls, *args, **kwargs):
        if not hasattr(cls, "_instance"):
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, width=224, height=224, cap_mode=0):
        cls = type(self)
        if not hasattr(cls, "_init"):
            self.width = width
            self.height = height

            self.image = np.empty((self.height, self.width, 3), dtype=np.uint8)

            try:
                cap_mode_table = {0:(3264, 2464, 21), 1:(3264, 1848, 28), 2:(1920, 1080, 30), 3:(1280, 720, 60), 4:(1280, 720, 120)}
                capture_width, capture_height, fps = cap_mode_table[cap_mode]

                gst_str = 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=%d, height=%d, format=(string)NV12, framerate=(fraction)%d/1 ! nvvidconv flip-method=%s ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! videoconvert ! appsink' % (capture_width, capture_height, fps, __main__._camera_flip_method, self.width, self.height)
                self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

                re, image = self.cap.read()
                if not re:
                    raise RuntimeError('Could not read image from camera.')

                self.image = image
                self.start()
            except:
                raise RuntimeError('Could not initialize camera.  Please see error trace.')

            cls._init = True
    
    def __del__(self):
        self.cap.release()

    def _capture_frames(self):
        while not self._is_stop :
            re, image = self.cap.read()
            if re:
                self.image = image
            else:
                raise RuntimeError('Could not read image from camera.')

    def start(self):
        if not hasattr(self, 'thread') or not self.thread.isAlive():
            self._is_stop = False
            self.thread = Thread(target=self._capture_frames)
            self.thread.daemon = True
            self.thread.start()

    def stop(self):
        if hasattr(self, 'thread'):
            self._is_stop = True
            self.thread.join()


class JupyterWidgetCamera(NewCamera):
    def __init__(self, width=224, height=224, fps=21, capture_width=3280, capture_height=2464):
        self.width = width
        self.height = height

        self.camera = Camera(width, height, fps, capture_width, capture_height)
        self.image = widgets.Image(format='jpeg', width=self.width, height=self.height)
        self.camera_link=None

        #os.system("echo soda | sudo -S systemctl restart nvargus-daemon")

    def __call__(self):
        self.camera_link.link()
        return self.image

    def show(self):
        if self.camera_link is None:
            self.camera_link = traitlets.dlink((self.camera, 'image'), (self.image, 'value'), transform=bgr8_to_jpeg)
        else:
            self.camera_link.link()

        display(self.image)

    def stop(self):
        if self.camera_link is not None:
            self.camera_link.unlink()

    @property
    def value(self):
        return self.camera.value

```
