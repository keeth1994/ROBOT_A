"""Optional diagnostic recording; disk and image encoding never run in control."""
import json
from pathlib import Path
import queue
import threading
import time

from PIL import Image


class ControlRecording:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open('x', encoding='utf-8')
        self.images = self.path.parent / (self.path.stem + '_frames')
        self.images.mkdir(exist_ok=True)
        self.queue = queue.Queue(maxsize=200)
        self.finished = threading.Event()
        self.dropped = 0
        self.error = None
        self.worker = threading.Thread(target=self._write, daemon=True)
        self.worker.start()

    def submit(self, record, camera_frame=None):
        if self.error or self.finished.is_set():
            return
        try:
            self.queue.put_nowait((record, camera_frame))
        except queue.Full:
            self.dropped += 1  # Recording must never delay robot control.

    def _write(self):
        last_sequence = None
        flushed = time.monotonic()
        try:
            while not self.finished.is_set() or not self.queue.empty():
                try:
                    record, camera = self.queue.get(timeout=.1)
                except queue.Empty:
                    if time.monotonic() - flushed >= 1:
                        self.stream.flush()
                        flushed = time.monotonic()
                    continue
                if camera is not None:
                    sequence, received, rgb = camera
                    name = f'{sequence:08d}.jpg'
                    if sequence != last_sequence:
                        Image.fromarray(rgb).save(self.images / name, quality=95)
                        last_sequence = sequence
                    record = record | {'camera_sequence': sequence, 'camera_received_s': received,
                                       'camera_image': f'{self.images.name}/{name}'}
                self.stream.write(json.dumps(record) + '\n')
                if time.monotonic() - flushed >= 1:
                    self.stream.flush()
                    flushed = time.monotonic()
            self.stream.write(json.dumps({'event': 'recording_closed', 'dropped': self.dropped}) + '\n')
        except Exception as error:
            self.error = str(error)
        finally:
            self.stream.close()

    def close(self):
        self.finished.set()
        self.worker.join(timeout=2)
