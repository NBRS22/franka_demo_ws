"""Proactive Agent — entry point."""

import logging
import signal

import uvicorn

from config import load_config
from app.server import create_app

logger = logging.getLogger(__name__)


def main():
    config = load_config()
    app = create_app(config)
    server = uvicorn.Server(
        uvicorn.Config(app, host="0.0.0.0", port=config.port, log_level="info")
    )

    def _shutdown(sig, frame):
        del sig, frame
        server.should_exit = True

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)
    server.run()


if __name__ == "__main__":
    main()
