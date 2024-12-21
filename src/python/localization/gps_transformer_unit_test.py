import math

import geometry
import pytest

from localization import gps_transformer, primitives

_UTM_ZONE = primitives.UTMZone(
    zone_number=17, hemisphere="north", epsg_code="EPSG:32617"
)
_BOCA_LATITUDE = 26.3353346
_BOCA_LONGITUDE = -80.128041
_BOCA_ELLIPSOID_HEIGHT = -19.261
_EXPECTED_POSITION = geometry.Position(
    geometry.UTM, 587016.4340624391, 2913114.641858696, _BOCA_ELLIPSOID_HEIGHT
)


@pytest.fixture
def transformer() -> gps_transformer.GPSTransformer:
    return gps_transformer.GPSTransformer(_UTM_ZONE)


@pytest.mark.parametrize("heading", [0.0, 90.0, 180])
def test_gps_transformer_heading(
    transformer: gps_transformer.GPSTransformer, heading: float
) -> None:
    utm_yaw = transformer.transform_heading(_BOCA_LONGITUDE, _BOCA_LATITUDE, heading)
    calculated_grid_convergence = _calc_grid_convergence(transformer)

    assert math.isclose(utm_yaw, 90 - (heading + calculated_grid_convergence))


def test_gps_transformer_position(transformer: gps_transformer.GPSTransformer) -> None:
    utm_position = transformer.transform_position(
        _BOCA_LONGITUDE, _BOCA_LATITUDE, ellipsoid_height=_BOCA_ELLIPSOID_HEIGHT
    )

    assert utm_position.is_close(_EXPECTED_POSITION)


def test_out_of_utm_zone(transformer: gps_transformer.GPSTransformer) -> None:
    with pytest.raises(ValueError):
        _ = transformer.transform_position(0.0, 0.0, ellipsoid_height=0.0)


def _calc_grid_convergence(transformer: gps_transformer.GPSTransformer) -> float:
    utm_position = transformer.transform_position(
        _BOCA_LONGITUDE, _BOCA_LATITUDE, ellipsoid_height=_BOCA_ELLIPSOID_HEIGHT
    )
    shifted_utm_position = transformer.transform_position(
        _BOCA_LONGITUDE,
        _BOCA_LATITUDE + 0.00001,
        ellipsoid_height=_BOCA_ELLIPSOID_HEIGHT,
    )

    easting_delta = shifted_utm_position.x - utm_position.x
    northing_delta = shifted_utm_position.y - utm_position.y
    return math.degrees(math.atan2(easting_delta, northing_delta))
