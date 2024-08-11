import sys

# For Windows
if sys.platform.startswith('win'):
    import msvcrt
# For Unix-based systems
else:
    import tty
    import termios

def get_key_press():
    """Get a single character from standard input, without echoing to the screen."""
    if sys.platform.startswith('win'):
        while True:
            if msvcrt.kbhit():
                ch = msvcrt.getch()
                return ch.decode()
    else:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
            if ord(ch) == 3:  # Ctrl+C
                raise KeyboardInterrupt
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
