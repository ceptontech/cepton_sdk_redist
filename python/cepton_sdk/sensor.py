import ctypes
import enum

import cepton_sdk.c
from cepton_sdk.common import *

_all_builder = AllBuilder(__name__)


class SensorModel(enum.IntEnum):
    HR80T = 1
    HR80M = 2
    HR80W = 3
    SORA_200 = 4
    VISTA_860 = 5
    HR80T_R2 = 6
    VISTA_860_GEN2 = 7
    FUSION_790 = 8
    VISTA_M = 9
    VISTA_X = 10
    SORA_P60 = 11
    VISTA_P60 = 12
    VISTA_X15 = 13
    VISTA_P90 = 14
    SORA_P90 = 15
    VISTA_P61 = 16
    SORA_P61 = 17
    VISTA_H120 = 18
    VISTA_P60_REV2 = 19


class SensorInformation(ToCMixin, ToDictMixin):
    """
    Attributes:
        handle
        serial_number
        model_name
        model (:class:`cepton_sdk.SensorModel`)
        firmware_version
        formal_firmware_version
        last_reported_temperature
        last_reported_humidity
        last_reported_age
        last_reported_hv
        last_reported_optic_temperature
        gps_ts_year
        gps_ts_mont
        gps_ts_day
        gps_ts_hour
        gps_ts_min
        gps_ts_sec
        return_count
        is_mocked
        is_pps_connected
        is_nmea_connected
        is_calibrated
        is_over_heated
    """

    @classmethod
    def _get_c_class(cls):
        return cepton_sdk.c.C_SensorInformation

    @classmethod
    def _get_dict_member_names(cls):
        return cls._get_c_member_names()

    __from_c_value_functions = {
        "model_name": lambda x: x.decode("utf-8"),
        "model": SensorModel,
        "firmware_version": lambda x: x.decode("utf-8"),
        "is_mocked": bool,
        "is_pps_connected": bool,
        "is_nmea_connected": bool,
        "is_ptp_connected": bool,
        "is_calibrated": bool,
        "is_over_heated": bool,
    }

    @classmethod
    def _from_c_value(cls, member_name, c_value):
        if member_name in cls.__from_c_value_functions:
            return cls.__from_c_value_functions[member_name](c_value)
        else:
            return super()._from_c_value(member_name, c_value)


def get_n_sensors():
    return int(cepton_sdk.c.c_get_n_sensors())


def get_sensor_handle(serial_number):
    c_sensor_handle = cepton_sdk.c.C_SensorHandle()
    cepton_sdk.c.c_get_sensor_handle_by_serial_number(
        serial_number, ctypes.byref(c_sensor_handle))
    return int(c_sensor_handle.value)


def get_sensor_information_by_index(sensor_index):
    c_sensor_info = cepton_sdk.c.C_SensorInformation()
    cepton_sdk.c.c_get_sensor_information_by_index(
        sensor_index, ctypes.byref(c_sensor_info))
    return SensorInformation.from_c(c_sensor_info)


def get_sensor_information_by_handle(sensor_handle):
    c_sensor_info = cepton_sdk.c.C_SensorInformation()
    cepton_sdk.c.c_get_sensor_information(
        sensor_handle, ctypes.byref(c_sensor_info))
    return SensorInformation.from_c(c_sensor_info)


def has_sensor(sensor_serial_number):
    try:
        get_sensor_handle(sensor_serial_number)
    except:
        return False
    return True


def get_sensor_information(sensor_serial_number):
    sensor_handle = get_sensor_handle(sensor_serial_number)
    return get_sensor_information_by_handle(sensor_handle)


__all__ = _all_builder.get()
