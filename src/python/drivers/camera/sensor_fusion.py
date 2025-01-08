from pyzed import sl  # type:ignore[import-untyped]


class SensorFusion:
    """A sensor fusion object, useful for pyzed and localization.
    NOTE: WIP code.
    """

    def __init__(self) -> None:
        self._fusion = sl.Fusion()
        init_fusion_param = sl.InitFusionParameters(coordinate_units=sl.UNIT.METER)
        fusion_init_code = self._fusion.init(init_fusion_param)
        if fusion_init_code != sl.FUSION_ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to initialize fusion: {fusion_init_code=}")

    def enable_fusion_position_tracking(self) -> bool:
        gnss_calibration_parameters = sl.GNSSCalibrationParameters(
            target_yaw_uncertainty=0.1,
            enable_translation_uncertainty_target=False,
            target_translation_uncertainty=15e-2,
            enable_reinitialization=True,
            gnss_vio_reinit_threshold=5,
        )
        positional_tracking_fusion_parameters = sl.PositionalTrackingFusionParameters(
            enable_GNSS_fusion=True,
            gnss_calibration_parameters=gnss_calibration_parameters,
        )
        status = self._fusion.enable_positionnal_tracking(
            positional_tracking_fusion_parameters
        )
        return status == sl.FUSION_ERROR_CODE.SUCCESS
