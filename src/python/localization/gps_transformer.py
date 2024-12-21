import math
from typing import Optional

import geometry
import pygeodesy  # type: ignore[import-untyped]
import pyproj
import system_info

from localization import primitives

_GEOID_FILEPATH = system_info.get_root_project_directory() / "env/geoids/egm2008-5.pgm"


class GPSTransformer:
    """Transforms GPS WGS84 to UTM (Universal Transverse Mercator) based on the passed
    zone. UTM divides the globe into 60 zones, each 6° wide in longitude. UTM provides
    metric coordinates in meters, making it suitable for local and regional navigation
    with high accuracy, especially for outdoor navigation in a limited geographic
    region.
    """

    def __init__(self, utm_zone: primitives.UTMZone):
        self._utm_zone = utm_zone
        # Initialize the Transformer using WGS84 (lat/lon) to UTM Zone
        self._transformer = pyproj.Transformer.from_crs(
            "EPSG:4326", utm_zone.epsg_code, always_xy=True
        )
        # Load a geoid model (e.g., EGM2008) at 5-minute resolution grid to convert
        # between height above sea level and the ellipsoidal height.
        self._egm_2008_geoid = pygeodesy.GeoidKarney(_GEOID_FILEPATH)

    def transform_position(
        self,
        longitude: float,
        latitude: float,
        *,
        ellipsoid_height: Optional[float] = None,
        height_above_sea_level: Optional[float] = None,
    ) -> geometry.Position:
        self._raise_if_not_in_utm_zone(longitude, latitude)
        if not ellipsoid_height:
            if height_above_sea_level:
                # Get geoid undulation at the given location
                position = pygeodesy.ellipsoidalKarney.LatLon(latitude, longitude)
                # The height of the geoid above the ellipsoid, N, is sometimes called
                # the geoid undulation. It can be used to convert a height above the
                # ellipsoid, h, to the corresponding height above the geoid (the
                # orthometric height, roughly the height above mean sea level), H, using
                # the relations
                # h = N + H;   H = −N + h.
                geoid_undulation = self._egm_2008_geoid([position])[0]
                # Calculate ellipsoidal height
                ellipsoid_height = height_above_sea_level + geoid_undulation
            else:
                # Default to zero to get a calculation even if its not as accurate.
                ellipsoid_height = 0.0

        # Perform the transformation
        utm_x, utm_y, utm_z = self._transformer.transform(
            longitude, latitude, ellipsoid_height
        )
        return geometry.Position(geometry.UTM, utm_x, utm_y, utm_z)

    def transform_heading(
        self, longitude: float, latitude: float, heading: float
    ) -> float:
        """Transforms the heading defined in ublox and WGS84 into one our system uses."""
        self._raise_if_not_in_utm_zone(longitude, latitude)
        # WGS84 heading and course are clockwise from north, but our convention is
        # counterclockwise from grid east
        grid_convergence_angle = self._grid_convergence(longitude, latitude)
        grid_heading = heading + grid_convergence_angle

        # WGS84 heading and course are clockwise from north, but our convention is
        # counterclockwise from grid east
        yaw = 90 - grid_heading

        return geometry.wrap_degrees(yaw)

    def _grid_convergence(self, longitude: float, latitude: float) -> float:
        """Calculates convergence angle by comparing two positions. The first being the
        actual position being evaluated and the other 1m north of that position.
        Evalutes the angle formed between the two points vs a true north heading
        (implied by moving 1m north) to understand the convergence angle between the
        two. Returned in degrees. Considers an ellipsoid height of zero since it doesnt
        affect the results.
        """
        x0, y0, _ = self._transformer.transform(longitude, latitude, 0.0)
        x1, y1, _ = self._transformer.transform(longitude, latitude + 0.00001, 0.0)
        easting_delta = x1 - x0
        northing_delta = y1 - y0
        return math.degrees(math.atan2(easting_delta, northing_delta))

    def _raise_if_not_in_utm_zone(self, longitude: float, latitude: float) -> None:
        cur_utm_zone = primitives.UTMZone.from_coordinates(longitude, latitude)
        if cur_utm_zone != self._utm_zone:
            raise ValueError(
                "GPS latitude / longitude not within the transformer's UTMZone"
                f"{cur_utm_zone=}, {self._utm_zone=}, {latitude=}, {longitude=}"
            )
