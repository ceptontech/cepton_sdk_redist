using System.Runtime.InteropServices;

namespace Cepton.SDK
{
    public static class CaptureReplay
    {
        #region Private interfaces and helpers
        private static void _E(SensorErrorCode ec)
        {
            if (ec != SensorErrorCode.SUCCESS)
                throw new CeptonSDKException(ec);
        }

        [DllImport("cepton_sdk.dll", EntryPoint = "cepton_sdk_capture_replay_open")]
        private static extern SensorErrorCode _Open(string path);

        [DllImport("cepton_sdk.dll", EntryPoint = "cepton_sdk_capture_replay_is_open")]
        private static extern bool _IsOpen();

        [DllImport("cepton_sdk.dll", EntryPoint = "cepton_sdk_capture_replay_close")]
        private static extern SensorErrorCode _Close();

        [DllImport("cepton_sdk.dll", EntryPoint = "cepton_sdk_capture_replay_get_start_time")]
        private static extern ulong _GetStartTime();

        [DllImport("cepton_sdk.dll", EntryPoint = "cepton_sdk_capture_replay_get_position")]
        private static extern float _GetStartPosition();

        [DllImport("cepton_sdk.dll", EntryPoint = "cepton_sdk_capture_replay_get_length")]
        private static extern float _GetLength();

        [DllImport("cepton_sdk.dll", EntryPoint = "cepton_sdk_capture_replay_is_end")]
        private static extern bool _IsEnd();

        [DllImport("cepton_sdk.dll", EntryPoint = "cepton_sdk_capture_replay_resume_blocking")]
        private static extern SensorErrorCode _ResumeBlocking(float duration);

        [DllImport("cepton_sdk.dll", EntryPoint = "cepton_sdk_capture_replay_resume")]
        private static extern SensorErrorCode _Resume();

        [DllImport("cepton_sdk.dll", EntryPoint = "cepton_sdk_capture_replay_pause")]
        private static extern SensorErrorCode _Pause();

        [DllImport("cepton_sdk.dll", EntryPoint = "cepton_sdk_capture_replay_rewind")]
        private static extern SensorErrorCode _Rewind();

        [DllImport("cepton_sdk.dll", EntryPoint = "cepton_sdk_capture_replay_seek")]
        private static extern SensorErrorCode _Seek(float position);

        [DllImport("cepton_sdk.dll", EntryPoint = "cepton_sdk_capture_replay_is_running")]
        private static extern bool _IsRunning();

        [DllImport("cepton_sdk.dll", EntryPoint = "cepton_sdk_capture_replay_set_enable_loop")]
        private static extern SensorErrorCode _EnableLoop(bool enabled);

        [DllImport("cepton_sdk.dll", EntryPoint = "cepton_sdk_capture_replay_get_enable_loop")]
        private static extern bool _IsLoopEnabled();

        [DllImport("cepton_sdk.dll", EntryPoint = "cepton_sdk_capture_replay_set_speed")]
        private static extern SensorErrorCode _SetSpeed(float speed);
        [DllImport("cepton_sdk.dll", EntryPoint = "cepton_sdk_capture_replay_get_speed")]
        private static extern float _GetSpeed();
        #endregion

        public static void Open(string path) { _E(_Open(path)); }
        public static void Close() { _E(_Close()); }
        public static void ResumeBlocking(float duration) { _E(_ResumeBlocking(duration)); }
        public static void Resume() { _E(_Resume()); }
        public static void Rewind() { _E(_Rewind()); }
        public static void Seek(float position) { _E(_Seek(position)); }
        public static void SeekRelative(float position) { Seek(StartPosition + position); }
        public static void EnableLoop(bool enabled) { }

        public static bool IsOpen => _IsOpen();
        public static bool IsEnd => _IsEnd();
        public static bool IsRunning => _IsRunning();
        public static bool LoopEnabled
        {
            get { return _IsLoopEnabled(); }
            set { _E(_EnableLoop(value)); }
        }
        public static float Speed
        {
            get { return _GetSpeed(); }
            set { _E(_SetSpeed(value)); }
        }

        public static ulong StartTime => _GetStartTime();
        public static float StartPosition => _GetStartPosition();
        public static float CaptureLength => _GetLength();
    }
}
