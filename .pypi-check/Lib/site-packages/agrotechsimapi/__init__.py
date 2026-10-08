"""AgroTechSim API. Import optional flight/vision dependencies on first use."""
from importlib import import_module

_EXPORTS = {
    'SimClient': ('client', 'SimClient'),
    'CaptureType': ('client', 'CaptureType'),
    'PID': ('pid', 'PID'), 'AdaptivePID': ('pid', 'AdaptivePID'),
    'HighLevelSimClient': ('high_level_client', 'HighLevelSimClient'),
    'HighLevelClient': ('high_level_client', 'HighLevelSimClient'),
    'CalibrationStore': ('calibration.store', 'CalibrationStore'),
    'Frame': ('video_pb2', 'Frame'), 'Response': ('video_pb2', 'Response'),
    'DESCRIPTOR': ('video_pb2', 'DESCRIPTOR'),
    **{name: ('video_pb2_grpc', name) for name in (
        'VideoStreamServiceStub', 'VideoStreamServiceServicer',
        'add_VideoStreamServiceServicer_to_server', 'VideoStreamService')},
    **{name: ('utils.utils', name) for name in (
        'LoopingTimer', 'sim_to_api_distance', 'vel_to_rc_signal')},
    **{name: ('utils.vision', name) for name in (
        'process_aruco', 'process_blob', 'resolution_changes')},
    **{name: ('utils.recognition_setting', name) for name in (
        'aruco_dictionary', 'detector_parameters', 'marker_size',
        'distance_coefficients', 'camera_matrix')},
}
__all__ = list(_EXPORTS)


def __getattr__(name):
    if name not in _EXPORTS:
        raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
    module, attribute = _EXPORTS[name]
    value = getattr(import_module('.' + module, __name__), attribute)
    globals()[name] = value
    return value
