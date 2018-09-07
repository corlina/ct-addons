from __future__ import absolute_import
import threading
import time
import socket
import struct
import logging
from ct_addons.event_trackers.mpu6050 import data_source, motion_tracker


log = logging.getLogger(__name__)


class Mpu6050EventTracker(object):

    EVENT_TYPE = 'corlina.mpu6050'

    def __init__(self, client, run_server_at_port=None):
        self.client = client
        self._stopped = threading.Event()
        generator = data_source.mpu6050_data_generator(0.011, self._stopped)
        generator = data_source.dump_to_file(generator, 'data.txt', 1000)
        generator = data_source.motiontracker_data_generator(
            generator,
            motion_tracker.MotionTracker(0.5, 0.011),
            calibrate_n=300,
        )
        self.streamer = data_source.DataStreamer(generator)
        self.streamer.add_consumer(self._react_for_epoch_condition)
        self._run_server_at_port = run_server_at_port

        self._lock = threading.Lock()
        self._config_state = False
        self._max_angle_deviation = 30.0
        self._is_in_epoch_condition = False

    def _react_for_epoch_condition(self,
                          accx, accy, accz,
                          gyrox, gyroy, gyroz,
                          anglex, angley, anglez):
        with self._lock:
            if self._config_state:
                return
        maxdev = max(abs(anglex), abs(angley), abs(anglez))
        now_in_condition = maxdev > self._max_angle_deviation
        need_epoch = now_in_condition != self._is_in_epoch_condition
        if need_epoch:
            log.info('met Epoch condition: %s', 'IN' if now_in_condition else 'OUT')
            if now_in_condition:
                data = {'x': anglex, 'y': angley, 'z': anglez}
                self.client.send_event('ORIENTATION', data)
        self._is_in_epoch_condition = now_in_condition

    def run(self):
        try:
            if self._run_server_at_port is None:
                while True:
                    time.sleep(1)
            else:
                run_server(self._run_server_at_port, self.streamer)
        finally:
            log.info('interrupted')
            self._stopped.set()
            self.streamer.wait_for_end()

    def on_config_enabled(self, etype, params):
        with self._lock:
            self._config_state = True
            self._max_angle_deviation = params.get('max_angle_deviation', self._max_angle_deviation)

    def on_config_disabled(self, etype, params):
        with self._lock:
            self._config_state = False


def run_server(port, streamer):
    serversock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serversock.bind(('0.0.0.0', port))
    serversock.listen(3)
    while True:
        sock, addr = serversock.accept()
        cons = ClientConsumer(sock, streamer)
        log.info('connected client: %s -> %r', addr, cons)
        cid = streamer.add_consumer(cons)
        cons.consumer_id = cid


class ClientConsumer(object):
    def __init__(self, sock, streamer):
        self.sock = sock
        self.streamer = streamer
        self.consumer_id = None

    def __call__(self, *data):
        packet = struct.pack('f' * len(data), *data)
        try:
            self.sock.send(packet)
        except socket.error:
            if self.consumer_id is not None:
                self.streamer.remove_consumer(self.consumer_id)
                self.consumer_id = None