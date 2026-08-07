import argparse
import sys
import time

import msgpack
import zmq


def _new_req_socket(context, host, port, timeout_ms):
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
    socket.setsockopt(zmq.LINGER, 0)  # don't block on close() if nothing ever answers
    socket.connect(f"tcp://{host}:{port}")
    return socket


def check_zmq_health(name, host, port, max_retries=10, retry_delay=2.0, timeout_ms=3000):
    context = zmq.Context()
    socket = _new_req_socket(context, host, port, timeout_ms)

    for attempt in range(max_retries):
        try:
            socket.send(msgpack.packb({"action": "health"}))
            result = msgpack.unpackb(socket.recv(), raw=False)
            if result.get("status") == "ok":
                print(f"[check_prerequisites] OK - {name} available at {host}:{port}")
                socket.close()
                context.term()
                return True

        except zmq.Again:
            print(
                f"[check_prerequisites] {name}: attempt {attempt + 1}/{max_retries} "
                f"failed, retrying in {retry_delay}s...",
                file=sys.stderr
            )
            socket.close()
            socket = _new_req_socket(context, host, port, timeout_ms)
            time.sleep(retry_delay)

    socket.close()
    context.term()
    print(f"[check_prerequisites] FAILED - {name} unavailable at {host}:{port}", file=sys.stderr)
    return False


# add an entry here to cover more external servers (Gemini ER, ...)
CHECKS = {
    "sam3": {"name": "SAM3 server", "host": "127.0.0.1", "port": 5557},
    "graspgen": {"name": "GraspGen server", "host": "127.0.0.1", "port": 5556},
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--only', choices=sorted(CHECKS), action='append')
    parser.add_argument('--max-retries', type=int, default=None)
    parser.add_argument('--retry-delay', type=float, default=None)
    args = parser.parse_args()

    selected = args.only if args.only else list(CHECKS)

    overrides = {}
    if args.max_retries is not None:
        overrides['max_retries'] = args.max_retries
    if args.retry_delay is not None:
        overrides['retry_delay'] = args.retry_delay

    all_ok = True
    for key in selected:
        if not check_zmq_health(**{**CHECKS[key], **overrides}):
            all_ok = False

    if not all_ok:
        print(
            "[check_prerequisites] One or more required external servers are "
            "not running. Aborting launch.",
            file=sys.stderr
        )
        sys.exit(1)

    print("[check_prerequisites] All prerequisites OK.")
    sys.exit(0)


if __name__ == '__main__':
    main()
