
class AnsiColor:
    END = '\033[0m'
    
    BLACK = '\033[30m'
    WHITE = '\033[97m'
    LIGHT_GRAY = '\033[37m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    RED = '\033[31m'
    YELLOW = '\033[33m'
    GREEN = '\033[92m'
    CYAN = '\033[96m'

    OK = GREEN
    WARN = YELLOW
    ERR = RED
    INFO = CYAN


def err(text):
    print(AnsiColor.ERR + text + AnsiColor.END)


def warn(text):
    print(AnsiColor.WARN + text + AnsiColor.END)


def ok(text):
    print(AnsiColor.OK + text + AnsiColor.END)


def info(text):
    print(AnsiColor.INFO + text + AnsiColor.END)