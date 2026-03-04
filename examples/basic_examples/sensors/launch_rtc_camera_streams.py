"""Launch chassis and head camera RTC streams for localhost viewing.

This helper enables selected camera sensors, switches RGB streams to RTC
transport, prints signaling URLs, and keeps the robot session alive so that
`rtc_stream_viewer.html` can connect from localhost.
"""

import time

import tyro
from dexbot_utils.configs.components.sensors.cameras import CameraConfig
from loguru import logger

from dexcontrol.core.config import get_robot_config
from dexcontrol.robot import Robot


CHASSIS_CAMERAS = [
    "base_left_camera",
    "base_right_camera",
    "base_front_camera",
    "base_back_camera",
]


def _ensure_chassis_camera_configs(configs) -> None:
    for camera_name in CHASSIS_CAMERAS:
        if camera_name not in configs.sensors:
            configs.sensors[camera_name] = CameraConfig(
                name=camera_name,
                enabled=True,
                transport="rtc",
                topic=f"sensors/{camera_name}/rgb",
                rtc_channel=f"sensors/{camera_name}/rgb_rtc",
            )
        else:
            configs.sensors[camera_name].enabled = True
            configs.sensors[camera_name].transport = "rtc"


def _enable_head_camera(configs) -> None:
    configs.enable_sensor("head_camera")
    head_cfg = configs.sensors["head_camera"]

    # Prefer RTC for RGB streams, keep depth on Zenoh.
    if hasattr(head_cfg, "left_rgb"):
        head_cfg.left_rgb.enabled = True
        head_cfg.left_rgb.transport = "rtc"
    if hasattr(head_cfg, "right_rgb"):
        head_cfg.right_rgb.enabled = True
        head_cfg.right_rgb.transport = "rtc"
    if hasattr(head_cfg, "depth"):
        head_cfg.depth.enabled = True
        head_cfg.depth.transport = "zenoh"


def _log_rtc_streams(sensor_name: str, sensor) -> None:
    info = getattr(sensor, "camera_info", None)
    if not info:
        logger.warning(f"{sensor_name}: camera_info not available")
        return

    streams = info.get("rtc", {}).get("streams", {})
    if not streams:
        logger.warning(f"{sensor_name}: no RTC streams found")
        return

    logger.info(f"{sensor_name} RTC streams:")
    for stream_name, stream_info in streams.items():
        signaling_url = stream_info.get("signaling_url", "N/A")
        codec = stream_info.get("codec", "N/A")
        logger.info(f"  - {stream_name}: {signaling_url} (codec={codec})")


def main(
    head_camera: bool = True,
    chassis_cameras: bool = True,
    print_head_obs_shapes_every_sec: float = 0.0,
) -> None:
    """Launch camera streams and keep session alive.

    Args:
        head_camera: Enable head ZED camera RTC RGB streams.
        chassis_cameras: Enable base chassis camera RTC streams.
        print_head_obs_shapes_every_sec: If > 0, periodically prints head camera
            observation shapes from get_obs(left_rgb, right_rgb, depth).
    """
    if not head_camera and not chassis_cameras:
        logger.error("Nothing selected. Enable at least one of head_camera/chassis_cameras.")
        return

    configs = get_robot_config()
    if chassis_cameras:
        _ensure_chassis_camera_configs(configs)
    if head_camera:
        _enable_head_camera(configs)

    with Robot(configs=configs) as robot:
        if chassis_cameras:
            for camera_name in CHASSIS_CAMERAS:
                sensor = getattr(robot.sensors, camera_name, None)
                if sensor and sensor.wait_for_active(timeout=8.0):
                    logger.success(f"{camera_name} active")
                    _log_rtc_streams(camera_name, sensor)
                else:
                    logger.warning(f"{camera_name} not active")

        if head_camera:
            sensor = getattr(robot.sensors, "head_camera", None)
            if sensor and sensor.wait_for_active(timeout=8.0):
                logger.success("head_camera active")
                _log_rtc_streams("head_camera", sensor)
            else:
                logger.warning("head_camera not active")

        logger.info("Streams are running. Open rtc_stream_viewer.html and connect to printed ws:// URLs.")
        logger.info("Press Ctrl+C to stop.")

        try:
            while True:
                if head_camera and print_head_obs_shapes_every_sec > 0:
                    camera_data = robot.sensors.head_camera.get_obs(
                        obs_keys=["left_rgb", "right_rgb", "depth"]
                    )
                    left = camera_data.get("left_rgb")
                    right = camera_data.get("right_rgb")
                    depth = camera_data.get("depth")
                    left_shape = getattr(left, "shape", None)
                    right_shape = getattr(right, "shape", None)
                    depth_shape = getattr(depth, "shape", None)
                    logger.info(
                        f"head_camera shapes: left={left_shape}, right={right_shape}, depth={depth_shape}"
                    )
                    time.sleep(print_head_obs_shapes_every_sec)
                else:
                    time.sleep(1.0)
        except KeyboardInterrupt:
            logger.info("Stopping stream launcher...")


if __name__ == "__main__":
    tyro.cli(main)
