import logging
import sys
import traceback

# --- ANSI Color Codes ---
_BLACK = "\033[30m"
_RED = "\033[91m"
_GREEN = "\033[92m"
_YELLOW = "\033[93m"
_BLUE = "\033[94m"
_MAGENTA = "\033[95m"
_CYAN = "\033[96m"
_WHITE = "\033[97m"
_RESET = "\033[0m"
_BOLD = "\033[1m"

# --- Log Level Configuration ---
_log_level = logging.INFO
_show_traceback = False


def set_log_level(level, show_traceback=False):
    """
    Sets the global log level for the print functions.

    Args:
        level (str or int): The desired log level, e.g., 'DEBUG', 'INFO', logging.DEBUG.
        show_traceback (bool): If True, print_error will include a traceback.
    """
    global _log_level, _show_traceback
    if isinstance(level, str):
        _log_level = getattr(logging, level.upper(), logging.INFO)
    else:
        _log_level = level
    _show_traceback = show_traceback


# --- Logger Setup ---
# Using a basic configuration. A more advanced setup could use handlers.
logging.basicConfig(level=logging.DEBUG, format="%(message)s", stream=sys.stdout)
_logger = logging.getLogger("mujoco_converter")


def _log(level, message, color=""):
    """Internal logging function."""
    if level >= _log_level:
        if color:
            _logger.log(level, f"{color}{message}{_RESET}")
        else:
            _logger.log(level, message)


# --- Public Print Functions ---


def print_base(message):
    """Prints a standard message."""
    _log(logging.INFO, message)


def print_info(message):
    """Prints an informational message in green."""
    _log(logging.INFO, message, _GREEN)


def print_warning(message):
    """Prints a warning message in yellow."""
    _log(logging.WARNING, f"[WARNING] {message}", _YELLOW)


def print_debug(message):
    """Prints a debug message in blue."""
    _log(logging.DEBUG, f"[DEBUG] {message}", _BLUE)


def print_error(message, exc_info=None):
    """
    Prints an error message in red.

    Args:
        message (str): The error message to print.
        exc_info (bool or Exception, optional): If True, prints the current exception's traceback.
                                                If an Exception object is passed, its traceback is printed.
                                                Defaults to None.
    """
    _log(logging.ERROR, f"[ERROR] {message}", _RED)

    # Determine if a traceback should be shown.
    # This can be triggered by the global flag or by passing exc_info.
    should_show_traceback = _show_traceback or exc_info

    if should_show_traceback:
        if isinstance(exc_info, BaseException):
            # If an exception object was passed, format and print it.
            tb_lines = traceback.format_exception(
                type(exc_info), exc_info, exc_info.__traceback__
            )
            sys.stderr.write("".join(tb_lines))
        else:
            # Otherwise, use the current system exception info.
            # This works inside an 'except' block.
            traceback.print_exc(file=sys.stderr)


def print_confirm(message):
    """Prints a confirmation message in magenta."""
    _log(logging.INFO, message, _MAGENTA)


def parse_actuator_gains(value):
    """
    Parse actuator gains from string format to dictionary.
    
    Supported formats:
    - "kp=500.0,kv=1.0" (comma-separated key=value pairs)
    - "kp=500.0 kv=1.0" (space-separated key=value pairs)
    - "kp=500.0,kv=1.0,dampratio=0.5" (with dampratio)
    - Dictionary input (passes through)
    - List input [kp, kv] (legacy format)
    
    Args:
        value: String, dict, or list representing actuator gains
    
    Returns:
        dict: Dictionary with parsed key-value pairs (values as floats)
        
    Raises:
        ValueError: If format is invalid or unknown keys are used
    """
    if isinstance(value, dict):
        return value
    
    if isinstance(value, list):
        # Legacy format: [kp, kv]
        if len(value) == 2:
            return {"kp": float(value[0]), "kv": float(value[1])}
        else:
            raise ValueError(f"Legacy list format must have exactly 2 values [kp, kv], got {len(value)}")
    
    result = {}
    # Try comma-separated first, then space-separated
    separators = [',', ' ']
    pairs = []
    
    for sep in separators:
        if sep in value:
            pairs = value.split(sep)
            break
    
    if not pairs:
        pairs = [value]
    
    for pair in pairs:
        pair = pair.strip()
        if not pair:
            continue
        
        if '=' not in pair:
            raise ValueError(f"Invalid format: '{pair}'. Expected format: key=value (e.g., 'kp=500.0,kv=1.0')")
        
        key, val = pair.split('=', 1)
        key = key.strip()
        val = val.strip()
        
        if key not in ['kp', 'kv', 'dampratio']:
            raise ValueError(f"Unknown actuator gain key: '{key}'. Allowed keys: kp, kv, dampratio")
        
        try:
            result[key] = float(val)
        except ValueError:
            raise ValueError(f"Invalid value for '{key}': '{val}'. Must be a number.")
    
    if not result:
        raise ValueError(f"No valid key=value pairs found in: '{value}'")
    
    return result