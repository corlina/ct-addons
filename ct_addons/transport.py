import socket
import json
import struct
import threading
import select
import logging


log = logging.getLogger(__name__)


class CTSocketClient(object):

    CT_AGENT_SOCKET_PATH = '/opt/corlina/var/event.sock'

    def __init__(self, client_id, event_types,
                 on_config_enabled, on_config_disabled,
                 socket_path=None):
        self.client_id = client_id
        self.event_types = event_types
        self.socket_path = socket_path or self.CT_AGENT_SOCKET_PATH
        self.on_config_enabled = on_config_enabled
        self.on_config_disabled = on_config_disabled
        self._sock = None
        self._interrupt_socks = socket.socketpair()
        self._thread = None
        self._stopped = threading.Event()

        # lock guards concurrent access on socket when reconnecting
        self._lock = threading.RLock()

    def start(self):
        if self._thread is not None:
            raise RuntimeError("Already started")
        self._stopped.clear()
        self._thread = threading.Thread(target=self._loop)
        self._thread.setDaemon(True)
        self._thread.start()

    def send_event(self, event_type, data):
        self._send({'event_type': event_type, 'data': data})

    def stop(self):
        self._stopped.set()
        self._interrupt_socks[1].write('\0')
        self._thread.join()
        self._interrupt_socks[0].read(1)
        self._thread = None

    def _reconnect(self):
        with self._lock:
            self._close_if_open()
            self._sock = socket.socket(socket.AF_UNIX)
            self._sock.setblocking(0)
            self._sock.connect(self.socket_path)

    def _send(self, contents):
        with self._lock:
            if self._sock is None:
                raise RuntimeError("Not connected")
            data = json.dumps(contents)
            header = struct.pack('>I', len(data))
            self._sock.send(header + data)

    def _send_hello(self):
        self._send({
            'client_id': self.client_id,
            'event_types': self.event_types,
        })

    def _loop(self):
        while not self._stopped.isSet():
            self._reconnect()
            self._send_hello()

            try:
                while not self._stopped.isSet():
                    self._process_one(self._read_one())
            except socket.error as err:
                log.error('Error while reading message: %s', err)
            except _ReadInterrupted:
                self._close_if_open()

    def _read_one(self):
        [msg_len] = struct.unpack('>I', self._read_data(_HEADER_LEN))
        data = self._read_data(msg_len)
        return json.loads(data)

    def _read_data(self, length):
        result = ''
        while len(result) < length and not self._stopped.isSet():
            rlist, _, _ = select.select([
                self._sock.fileno(),
                self._interrupt_socks[0].fileno()
            ], [], [])
            if self._interrupt_socks[0].fileno() in rlist:
                raise _ReadInterrupted
            result += self._sock.read(length - len(result))
        return result

    def _process_one(self, contents):
        cfg_enabled = contents['config_state_enabled']
        event_type = contents['event_type']
        options = contents['options']

        if cfg_enabled:
            if callable(self.on_config_enabled):
                self.on_config_enabled(event_type, options)
        else:
            if callable(self.on_config_disabled):
                self.on_config_disabled(event_type, options)

    def _close_if_open(self):
        with self._lock:
            if self._sock is not None:
                self._sock.close()
                self._sock = None


class _ReadInterrupted(Exception):
    pass


_HEADER_LEN = struct.calcsize('>I')

