import sys

# import ogl_viewer.viewer as gl
from pyzed import sl  # type:ignore[import-untyped]


def main() -> None:
    print(
        "Running Depth Sensing sample ... Press 'Esc' to quit\nPress 's' to save the point cloud"
    )

    camera1_params = sl.InitParameters(
        depth_mode=sl.DEPTH_MODE.ULTRA,
        coordinate_units=sl.UNIT.METER,
        coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP,
    )
    camera1_params.set_from_serial_number(43740003)
    camera1 = sl.Camera()
    status1 = camera1.open(camera1_params)
    camera2_params = sl.InitParameters(
        depth_mode=sl.DEPTH_MODE.ULTRA,
        coordinate_units=sl.UNIT.METER,
        coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP,
    )
    camera2_params.set_from_serial_number(45235829)
    camera2 = sl.Camera()
    status2 = camera2.open(camera2_params)

    if status1 != sl.ERROR_CODE.SUCCESS:
        print(repr(status1))
        sys.exit()
    if status2 != sl.ERROR_CODE.SUCCESS:
        print(repr(status2))
        sys.exit()

    res = sl.Resolution()
    res.width = 720
    res.height = 404

    # camera_model = camera1.get_camera_information().camera_model
    # Create OpenGL viewer
    # viewer = gl.GLViewer()
    # viewer.init(1, sys.argv, camera_model, res)

    # point_cloud = sl.Mat(res.width, res.height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU)

    # while viewer.is_available():
    #     if camera1.grab() == sl.ERROR_CODE.SUCCESS:
    #         camera1.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, res)
    #         viewer.updateData(point_cloud)
    #         if viewer.save_data:
    #             point_cloud_to_save = sl.Mat()
    #             camera1.retrieve_measure(
    #                 point_cloud_to_save, sl.MEASURE.XYZRGBA, sl.MEM.CPU
    #             )
    #             err = point_cloud_to_save.write("Pointcloud.ply")
    #             if err == sl.ERROR_CODE.SUCCESS:
    #                 print("Current .ply file saving succeed")
    #             else:
    #                 print("Current .ply file failed")
    #             viewer.save_data = False
    # viewer.exit()
    camera1.close()
    camera2.close()


if __name__ == "__main__":
    main()
