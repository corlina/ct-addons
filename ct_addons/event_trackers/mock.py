import threading
import time


class MockEventTracker(object):

    EVENT_TYPE = 'mock'

    def __init__(self, client, period):
        self.period = period
        self.client = client
        self._lock = threading.Lock()
        self._counter = 0
        self._config_state = False

    def run(self):
        while True:
            time.sleep(self.period)
            with self._lock:
                if self._config_state:
                    continue
                self._counter += 1
            self.client.send_event(self.EVENT_TYPE, {'counter': self._counter})

    def on_config_enabled(self, etype, params):
        with self._lock:
            self._config_state = True
            self._counter = params.get('counter', 0)

    def on_config_disabled(self, etype, params):
        with self._lock:
            self._config_state = False
