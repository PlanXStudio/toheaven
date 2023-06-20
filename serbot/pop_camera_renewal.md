# ~/pop/__init__.py
```pyton
def bgr8_to_jpeg(value):
    return bytes(cv2.imencode('.jpg', value)[1])

class _camera(SingletonConfigurable):
    value = traitlets.Any()

    width = traitlets.Integer(default_value=224).tag(config=True)
    height = traitlets.Integer(default_value=224).tag(config=True)
    fps = traitlets.Integer(default_value=21).tag(config=True)
    capture_width = traitlets.Integer(default_value=3280).tag(config=True)
    capture_height = traitlets.Integer(default_value=2464).tag(config=True)

    def __init__(self, *args, **kwargs):
        self.value = np.empty((self.height, self.width, 3), dtype=np.uint8)
        super(_camera, self).__init__(*args, **kwargs)
        
        self._is_stop = False

        try:
            self.cap = cv2.VideoCapture(self._gst_str(), cv2.CAP_GSTREAMER)

            re, image = self.cap.read()
            if not re:
                raise RuntimeError('Could not read image from camera.')

            self.value = image
            self.start()
        except:
            self.stop()
            raise RuntimeError('Could not initialize camera.  Please see error trace.')

        atexit.register(self.stop)

    def __del__(self):
        self._is_stop = True

    def _capture_frames(self):
        while not self._is_stop :
            re, image = self.cap.read()
            if re:
                self.value = image
            else:
                raise RuntimeError('Could not read image from camera.')
        print(">>> terminate _capture_frames")

    def _gst_str(self):
        return 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=%d, height=%d, format=(string)NV12, framerate=(fraction)%d/1 ! nvvidconv flip-method=%s ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! videoconvert ! appsink' % (self.capture_width, self.capture_height, self.fps, __main__._camera_flip_method, self.width, self.height)

    def start(self):
        if not self.cap.isOpened():
            self.cap.open(self._gst_str(), cv2.CAP_GSTREAMER)
        if not hasattr(self, 'thread') or not self.thread.isAlive():
            self.thread = Thread(target=self._capture_frames)
            self.thread.daemon = True
            self.thread.start()
        
        print("start thread")

    def stop(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        
        if hasattr(self, 'thread'):
            self._is_stop = True
            self.thread.join()
        
        print(">>> stop")

    def restart(self):
        self.stop()
        self.start()

class _camera2(SingletonConfigurable):
    cap_mode_table = {0:(3264, 2464, 21), 1:(3264, 1848, 28), 2:(1920, 1080, 30), 3:(1280, 720, 60), 4:(1280, 720, 120)}

    value = traitlets.Any()
    width = traitlets.Integer(default_value=224).tag(config=True)
    height = traitlets.Integer(default_value=224).tag(config=True)
    cap_mode = traitlets.Integer(default_value=0).tag(config=True)

    def __init__(self, *args, **kwargs):
        super(_camera2, self).__init__(*args, **kwargs)

        os.system("echo soda | sudo -S systemctl restart nvargus-daemon")

        capture_width, capture_height, fps = self.cap_mode_table[self.cap_mode]
        gst_str = 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=%d, height=%d, format=(string)NV12, framerate=(fraction)%d/1 ! nvvidconv flip-method=%s ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! videoconvert ! appsink' % (capture_width, capture_height, fps, __main__._camera_flip_method, self.width, self.height)

        self.value = np.empty((self.height, self.width, 3), dtype=np.uint8)
        try:
            self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

            success, tmp = self.cap.read()
            if not success:
                raise RuntimeError('Could not read image from camera.')

            self.start()
        except:
            raise RuntimeError('Could not initialize camera.  Please see error trace.')

        atexit.register(self.stop)

    def __del__(self):
        self.stop()
        self.cap.release()

    def _capture_frames(self):
        while not self._is_stop :
            success, self.value = self.cap.read()
            if not success:
                raise RuntimeError('Could not read image from camera.')

    def start(self):
        if not hasattr(self, '_thread') or not self._thread.isAlive():
            self._is_stop = False
            self._thread = Thread(target=self._capture_frames)
            self._thread.start()
        
    def stop(self):       
        if hasattr(self, '_thread'):
            self._is_stop = True
            self._thread.join()
            self.value = np.empty((self.height, self.width, 3), dtype=np.uint8)

    def restart(self):
        self.stop()
        self.start()


import atexit
import logging
import os
import time

class Camera2:
    _ref_count = 0
    log = logging.getLogger(__name__)
    cap_mode_table = {0:(3264, 2464, 21), 1:(3264, 1848, 28), 2:(1920, 1080, 30), 3:(1280, 720, 60), 4:(1280, 720, 120)}

    def __new__(cls, *args, **kwargs):       
        if not hasattr(cls, "_instance"):
            cls._instance = super().__new__(cls)

            logging.basicConfig(
                #filename = 'pop.log',
                level    = logging.INFO,
                format   = ">>> %(asctime)s - %(message)s"
            )

        return cls._instance

    def __init__(self, width=224, height=224, cap_mode=0):
        cls = type(self)
        if not hasattr(cls, "_init"):
            os.system("echo soda | sudo -S systemctl restart nvargus-daemon")

            self.width = width
            self.height = height

            try:
                capture_width, capture_height, fps = self.cap_mode_table[cap_mode]

                gst_str = 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=%d, height=%d, format=(string)NV12, framerate=(fraction)%d/1 ! nvvidconv flip-method=%s ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! videoconvert ! appsink' % (capture_width, capture_height, fps, __main__._camera_flip_method, self.width, self.height)
                self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

                success, tmp = self.cap.read()
                self._frame = np.empty((self.height, self.width, 3), dtype=np.uint8)

                if not success:
                    raise RuntimeError('Could not read image from camera.')
            except:
                raise RuntimeError('Could not initialize camera.')
                    
            cls._init = True
            atexit.register(self.__del__)
    
    def __del__(self):
        cls = type(self)
        if not hasattr(cls, "_del"):
            self.stop()
            self.cap.release()
            self.log.info("call __del__")

    def _capture_frames(self):
        self.log.info("enter _capture_frames")

        while not self._is_stop :
            success, self._frame = self.cap.read()
            if not success:
                raise RuntimeError('Could not read image from camera.')
            cv2.waitKey(1)

        self.log.info("leave _capture_frames")
        
    @property
    def value(self):
        return self._frame

    def start(self):
        self._ref_count += 1
        self.log.info("start ref_count %d", self._ref_count)
        
        if not hasattr(self, '_thread') or not self._thread.isAlive():
            self._is_stop = False
            self._thread = Thread(target=self._capture_frames)
            self._thread.start()

    def stop(self):
        self._ref_count -= 1
        self.log.info("stop ref_count %d", self._ref_count)

        if hasattr(self, '_thread') and self._ref_count <= 0:
            self._is_stop = True
            self._thread.join()
            self._frame = np.empty((self.height, self.width, 3), dtype=np.uint8)


from IPython.display import display, Image
import ipywidgets.widgets as widgets

class IPyCamera:
    code="7377EF6E7F5DC659B18B3A089F8BD812"

    def __init__(self, width=224, height=224, auto_load=True):
        self.width=width
        self.height=height
        
        if auto_load:
            self.load()

        """
        self.stopButton = widgets.ToggleButton(
            value=False,
            description='Stop',
            disabled=False,
            button_style='danger', # 'success', 'info', 'warning', 'danger' or ''
            tooltip='Description',
            icon='square' # (FontAwesome names without the `fa-` prefix)
        )
        """

    def __call__(self):
        self.camera_link.link()
        return self.value

    def load(self):
        self.camera = Camera2(self.width, self.height)
        self.image = widgets.Image(format='jpeg', width=self.width, height=self.height)
        self.camera_link = traitlets.dlink((self.camera, 'value'), (self.image, 'value'), transform=bgr8_to_jpeg)


    def _show(self, button):
        display_handle=display(None, display_id=True)

        while True:
            frame = self.camera.value
            frame = cv2.flip(frame, 1)
            _, frame = cv2.imencode('.jpeg', frame)
            display_handle.update(Image(data=frame.tobytes()))
            if self.stopButton.value == True:
                self.camera.stop()
                display_handle.update(None)    

    def show(self):
        """
        display(self.stopButton)
        self.camera.start()
        thread = Thread(target=self._show, args=(self.stopButton,))
        thread.start()
        """
        if self.camera_link is None:
            self.camera_link = traitlets.dlink((self.camera, 'value'), (self.image, 'value'), transform=bgr8_to_jpeg)
            display(self.image)
        else:
            self.camera_link.link()
            display(self.image)

    def stop(self):
        if self.camera_link is not None:
            self.camera_link.unlink()

    @property
    def value(self):
        return self.camera.value


class Camera:
    code="7377EF6E7F5DC659B18B3A089F8BD812"
    camera=None
    camera_link=None
    image=None
    width=None
    height=None

    def __init__(self, width=224, height=224, auto_load=True):
        self.width=width
        self.height=height

        if auto_load:
            self.load()


    def __call__(self):
        self.camera_link.link()
        return self.image

    def load(self):
        os.system("echo soda | sudo -S systemctl restart nvargus-daemon")
        self.camera = _camera.instance(width=self.width, height=self.height)
        self.image = widgets.Image(format='jpeg', width=self.width, height=self.height)
        self.camera_link = traitlets.dlink((self.camera, 'value'), (self.image, 'value'), transform=bgr8_to_jpeg)

    def show(self):
        if self.camera is None:
            self.load()

        if self.camera_link is None:
            self.camera_link = traitlets.dlink((self.camera, 'value'), (self.image, 'value'), transform=bgr8_to_jpeg)
            display(self.image)
        else:
            self.camera_link.link()
            display(self.image)

    def stop(self):
        if self.camera_link is not None:
            self.camera_link.unlink()

    @property
    def value(self):
        return self.camera.value

class IPyCamera2:
    code="7377EF6E7F5DC659B18B3A089F8BD812"
    camera=None
    camera_link=None
    image=None
    width=None
    height=None

    def __init__(self, width=224, height=224, cap_mode=0, auto_load=True):
        self.width=width
        self.height=height
        self.cap_mode=cap_mode

        if auto_load:
            self.load()

    def destroy(self):
        self.camera.stop()
        self.camera.cap.release()

    def __call__(self):
        self.camera_link.link()
        return self.image

    def load(self):
        self.camera = _camera2.instance(width=self.width, height=self.height, cap_mode=self.cap_mode)
        self.image = widgets.Image(format='jpeg', width=self.width, height=self.height)
        self.camera_link = traitlets.dlink((self.camera, 'value'), (self.image, 'value'), transform=bgr8_to_jpeg)

    def show(self):
        if self.camera is None:
            self.load()

        if self.camera_link is None:
            self.camera_link = traitlets.dlink((self.camera, 'value'), (self.image, 'value'), transform=bgr8_to_jpeg)
            display(self.image)
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
