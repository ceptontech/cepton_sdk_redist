using System;
using System.Runtime.InteropServices;

namespace Cepton.SDK
{
    public enum SensorErrorCode : int
    {
        SUCCESS = 0,
        ERROR_GENERIC = -1,
        ERROR_OUT_OF_MEMORY = -2,
        ERROR_SENSOR_NOT_FOUND = -4,
        ERROR_SDK_VERSION_MISMATCH = -5,
        ERROR_COMMUNICATION = -6,  ///< Networking error
        ERROR_TOO_MANY_CALLBACKS = -7,
        // Invalid value or uninitialized struct
        ERROR_INVALID_ARGUMENTS = -8,
        ERROR_ALREADY_INITIALIZED = -9,
        ERROR_NOT_INITIALIZED = -10,
        ERROR_INVALID_FILE_TYPE = -11,
        ERROR_FILE_IO = -12,
        ERROR_CORRUPT_FILE = -13,
        ERROR_NOT_OPEN = -14,
        ERROR_EOF = -15,

        FAULT_INTERNAL = -1000,  ///< Internal parameter out of range
        FAULT_EXTREME_TEMPERATURE = -1001,  ///< Reading exceed spec
        FAULT_EXTREME_HUMIDITY = -1002,     ///< Reading exceeds spec
        FAULT_EXTREME_ACCELERATION = -1003,
        FAULT_ABNORMAL_FOV = -1004,
        FAULT_ABNORMAL_FRAME_RATE = -1005,
        FAULT_MOTOR_MALFUNCTION = -1006,
        FAULT_LASER_MALFUNCTION = -1007,
        FAULT_DETECTOR_MALFUNCTION = -1008,
    };

    public class CeptonSDKException : Exception
    {
        public CeptonSDKException(SensorErrorCode ec)
        {
            ErrorCode = ec;
        }

        public SensorErrorCode ErrorCode { get; private set; }

        [DllImport("cepton_sdk.dll", EntryPoint = "cepton_is_error_code")]
        private static extern int _isError(int ec);

        [DllImport("cepton_sdk.dll", EntryPoint = "cepton_is_fault_code")]
        private static extern int _isFault(int ec);

        [DllImport("cepton_sdk.dll", EntryPoint = "cepton_get_error_code_name")]
        private static extern string _getName(int ec);

        public bool IsError { get { return _isError((int)ErrorCode) != 0; } }
        public bool IsFault { get { return _isFault((int)ErrorCode) != 0; } }
        public string Name { get { return _getName((int)ErrorCode); } }
    }
}
