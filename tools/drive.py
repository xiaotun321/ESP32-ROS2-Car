"""Send a bounded-duration motion command with the firmware's required heartbeat."""
import argparse
import math
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='e.g. COM5')
    parser.add_argument('--command', default='pwm 80 80',
                        help='w/s/a/d/x/stop or pwm LEFT RIGHT (open loop)')
    parser.add_argument('--seconds', type=float, default=0.3,
                        help='motion duration, >0 and <=30 seconds (default: 0.3)')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 30:
        parser.error('--seconds must be >0 and <=30')
    tokens = args.command.split()
    valid = len(tokens) == 1 and tokens[0] in ('w', 's', 'a', 'd', 'x', 'stop')
    if len(tokens) == 3 and tokens[0] == 'pwm':
        try:
            valid = all(math.isfinite(float(v)) and abs(float(v)) <= 1e6 for v in tokens[1:])
        except ValueError:
            valid = False
    if not valid:
        parser.error('invalid motion command')
    try:
        import serial
    except ImportError:
        parser.error('pyserial is required: python -m pip install pyserial')
    command = (' '.join(tokens) + '\n').encode('ascii')
    # Explicit DTR/RTS levels avoid intentionally holding the board in reset.
    with serial.Serial(port=None, baudrate=115200, timeout=0.05, write_timeout=0.2) as link:
        link.dtr = False
        link.rts = False
        link.port = args.port
        link.open()
        time.sleep(2)  # Allow boards that reset on opening the port to boot.
        try:
            link.write(b'stop\nstream 1\n')
            deadline = time.monotonic() + args.seconds
            next_send = 0
            pending = ''
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_send:
                    link.write(command)
                    next_send = now + 0.1
                if link.in_waiting:
                    response = link.read(link.in_waiting).decode('ascii', errors='replace')
                    print(response, end='', flush=True)
                    pending += response
                    while '\n' in pending:
                        line, pending = pending.split('\n', 1)
                        if line.startswith(('FAULT ', 'ERR ')):
                            raise RuntimeError('Firmware rejected motion or detected a fault: ' + line)
                time.sleep(0.01)
        finally:
            link.write(b'stop\nstream 0\nstatus\n')
            time.sleep(0.1)
            print(link.read(link.in_waiting).decode('ascii', errors='replace'), end='')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nStopped.')
