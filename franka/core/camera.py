import logging
import threading
import time

import zmq

logger = logging.getLogger(__name__)


class ZmqCamera:
    """Background thread subscribing to a ZMQ PUB camera stream.

    Expects multipart messages [b"rgb", <jpeg_bytes>].
    """

    def __init__(self, url: str):
        self._url = url
        self._latest: bytes | None = None
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._running = False

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()
        logger.info("ZmqCamera started: %s", self._url)

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        logger.info("ZmqCamera stopped.")

    @property
    def latest_jpeg(self) -> bytes | None:
        with self._lock:
            return self._latest

    @property
    def has_frame(self) -> bool:
        return self._latest is not None

    def _recv_loop(self) -> None:
        ctx = zmq.Context()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.RCVHWM, 1)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.SUBSCRIBE, b"rgb")
        sock.connect(self._url)
        logger.info("ZmqCamera connected to %s", self._url)

        frames_received = 0
        frames_dropped = 0
        last_stat_time = time.monotonic()

        try:
            while self._running:
                # Drain to latest: vider le socket et ne garder que la frame la plus récente.
                latest = None
                drained = 0
                while True:
                    try:
                        parts = sock.recv_multipart(flags=zmq.NOBLOCK)
                        if len(parts) >= 2:
                            if latest is not None:
                                drained += 1
                            latest = parts[1]
                    except zmq.Again:
                        break

                if latest is not None:
                    with self._lock:
                        self._latest = latest
                    frames_received += 1
                    frames_dropped += drained
                else:
                    time.sleep(0.005)

                now = time.monotonic()
                if now - last_stat_time >= 10.0:
                    elapsed = now - last_stat_time
                    fps = frames_received / elapsed
                    logger.info(
                        "ZmqCamera stats — consumed fps: %.1f, frames dropped by drain: %d",
                        fps,
                        frames_dropped,
                    )
                    frames_received = 0
                    frames_dropped = 0
                    last_stat_time = now
        finally:
            sock.close()
            ctx.term()
            logger.info("ZmqCamera recv loop exited.")
