# Gripper state codes
GRIPPER_STATES = {
    0: "IDLE",
    1: "GRASPING",
    2: "NO PART",
    3: "PART LOST",
    4: "HOLDING",
    5: "RELEASING",
    6: "POSITIONING",
    7: "ERROR"
}

# Error codes as per WSG documentation
ERROR_CODES = {
    0: "E_SUCCESS: No error occurred, operation was successful",
    1: "E_NOT_AVAILABLE: Function or data is not available",
    2: "E_NO_SENSOR: No measurement converter is connected",
    3: "E_NOT_INITIALIZED: Device was not initialized",
    4: "E_ALREADY_RUNNING: The data acquisition is already running",
    5: "E_FEATURE_NOT_SUPPORTED: The requested feature is currently not available",
    6: "E_INCONSISTENT_DATA: One or more parameters are inconsistent",
    7: "E_TIMEOUT: Timeout error",
    8: "E_READ_ERROR: Error while reading data",
    9: "E_WRITE_ERROR: Error while writing data",
    10: "E_INSUFFICIENT_RESOURCES: No more memory available",
    11: "E_CHECKSUM_ERROR: Checksum error",
    12: "E_NO_PARAM_EXPECTED: A parameter was given, but none expected",
    13: "E_NOT_ENOUGH_PARAMS: Not enough parameters to execute the command",
    14: "E_CMD_UNKNOWN: Unknown command",
    15: "E_CMD_FORMAT_ERROR: Command format error",
    16: "E_ACCESS_DENIED: Access denied",
    17: "E_ALREADY_OPEN: Interface is already open",
    18: "E_CMD_FAILED: Error while executing a command",
    19: "E_CMD_ABORTED: Command execution was aborted by the user",
    20: "E_INVALID_HANDLE: Invalid handle",
    21: "E_NOT_FOUND: Device or file not found",
    22: "E_NOT_OPEN: Device or file not open",
    23: "E_IO_ERROR: Input/Output Error",
    24: "E_INVALID_PARAMETER: Wrong parameter",
    25: "E_INDEX_OUT_OF_BOUNDS: Index out of bounds",
    26: "E_CMD_PENDING: Command was not completed yet",
    27: "E_OVERRUN: Data overrun",
    28: "RANGE_ERROR: Range error",
    29: "E_AXIS_BLOCKED: Axis blocked",
    30: "E_FILE_EXISTS: File already exists"
}
