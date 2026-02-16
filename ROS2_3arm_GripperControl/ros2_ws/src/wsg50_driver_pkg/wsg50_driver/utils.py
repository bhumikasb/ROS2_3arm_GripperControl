from wsg50_driver_pkg.wsg50_driver.constants import GRIPPER_STATES

def parse_grip_state(state_int):
    return GRIPPER_STATES.get(state_int, "UNKNOWN")

def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

def exponential_backoff(retries, base=0.5, factor=2.0, max_delay=10):
    delay = base * (factor ** retries)
    return min(delay, max_delay)