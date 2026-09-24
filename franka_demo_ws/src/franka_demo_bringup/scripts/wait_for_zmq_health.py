#!/usr/bin/env python3
"""Poll a ZMQ REQ/REP server with {'action': 'health'} until it answers {'status': 'ok'}."""
import argparse
import sys
import time

import msgpack
import zmq


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', required=True)
    parser.add_argument('--host', required=True)
    parser.add_argument('--port', type=int, required=True)
    parser.add_argument('--timeout', type=float, default=180.0)
    parser.add_argument('--retry-interval', type=float, default=2.0)
    args = parser.parse_args()

    context = zmq.Context()
    deadline = time.monotonic() + args.timeout

    while time.monotonic() < deadline:
        socket = context.socket(zmq.REQ)
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.RCVTIMEO, int(args.retry_interval * 1000))
        socket.connect(f'tcp://{args.host}:{args.port}')
        try:
            socket.send(msgpack.packb({'action': 'health'}))
            result = msgpack.unpackb(socket.recv(), raw=False)
            if result.get('status') == 'ok':
                print(f'[wait_for_zmq_health] {args.name} is up on {args.host}:{args.port}')
                return 0
            print(f'[wait_for_zmq_health] {args.name} unexpected status: {result}')
        except zmq.Again:
            print(f'[wait_for_zmq_health] waiting for {args.name} on {args.host}:{args.port}...')
        finally:
            socket.close(linger=0)

    print(f'[wait_for_zmq_health] TIMEOUT waiting for {args.name} on {args.host}:{args.port}', file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())
