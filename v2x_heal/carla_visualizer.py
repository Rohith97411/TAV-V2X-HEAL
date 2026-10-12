import datetime
import json
import math
import os
import random
import threading
import time
import urllib.request
import uuid

import carla
import numpy as np
import pygame

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class V2XHealCarlaVisualizer(Node):

    def __init__(self):
        super().__init__("v2x_heal_carla_visualizer")

        # ============================================================
        # ROS PARAMETERS
        # ============================================================
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 2000)
        self.declare_parameter("map_name", "Town04")
        self.declare_parameter("demo_duration_sec", 60.0)

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.map_name = str(self.get_parameter("map_name").value)
        self.demo_duration_sec = float(
            self.get_parameter("demo_duration_sec").value
        )

        # ============================================================
        # TAV INFORMATION
        # ============================================================
        self.vehicle_name = "Toyota Prius"
        self.demo_vin = "TAV-PRIUS-DEMO-001"

        self.tav_run_id = (
            "TAV-" + uuid.uuid4().hex[:8].upper()
        )

        # CARLA stock vehicle used as simulation surrogate.
        self.carla_vehicle_asset = "Toyota Prius"

        self.logo_url = os.environ.get(
            "TAV_LOGO_URL",
            "https://raw.githubusercontent.com/"
            "Thiyag15/tav-assets/main/logotav.png"
        )

        self.webhook_url = os.environ.get(
            "UIPATH_WEBHOOK_URL",
            ""
        ).strip()

        # ============================================================
        # FAULT STATE
        # ============================================================
        self.fault_active = False
        self.latest_fault_yaw = 0.0

        self.raw_camera_yaw_deg = 0.0
        self.software_correction_deg = 0.0
        self.effective_yaw_deg = 0.0

        self.fault_detected = False
        self.recovery_complete = False
        self.webhook_sent = False

        self.fault_start_time = None

        self.phase = "NORMAL"
        self.status_text = "ADAS CAMERA HEALTHY"

        # Show the raw fault for a short period.
        self.fault_hold_sec = 2.0

        # Time taken for simulated software compensation.
        self.calibration_duration_sec = 4.0

        # ============================================================
        # CARLA
        # ============================================================
        self.client = None
        self.world = None
        self.carla_map = None

        self.vehicle = None

        self.front_camera = None
        self.chase_camera = None

        self.traffic_manager = None
        self.autopilot_enabled = False

        # ============================================================
        # CAMERA DATA
        # ============================================================
        self.camera_width = 1280
        self.camera_height = 720

        self.front_frame = None
        self.chase_frame = None

        # ============================================================
        # DISPLAY
        # ============================================================
        self.display_width = 1280
        self.main_view_height = 680
        self.status_panel_height = 180
        self.display_height = (
            self.main_view_height
            + self.status_panel_height
        )

        self.pygame_enabled = False

        self.screen = None
        self.font_large = None
        self.font_medium = None
        self.font_small = None
        self.font_tiny = None

        self._setup_pygame()

        # ============================================================
        # CONNECT TO CARLA
        # ============================================================
        self._connect_to_carla()

        # ============================================================
        # ROS SUBSCRIPTIONS
        # ============================================================
        self.create_subscription(
            Bool,
            "/v2x_heal/fault_active",
            self.fault_state_callback,
            10
        )

        self.create_subscription(
            Float32,
            "/v2x_heal/camera_yaw_fault_deg",
            self.fault_yaw_callback,
            10
        )

        # ============================================================
        # MAIN TIMER
        # ============================================================
        self.demo_start_time = time.monotonic()

        self.done = False

        self.timer = self.create_timer(
            0.05,
            self.update_demo
        )

        # ============================================================
        # STARTUP LOGS
        # ============================================================
        self.get_logger().info("=" * 70)

        self.get_logger().info(
            "TAV | TOYOTA ADAS VERIFICATION - PROJECT PROTOTYPE"
        )

        self.get_logger().info(
            "V2X-HEAL FAIL-OPERATIONAL CARLA DEMONSTRATION"
        )

        self.get_logger().info(
            f"TAV RUN ID: {self.tav_run_id}"
        )

        self.get_logger().info(
            f"Target Vehicle Profile: {self.vehicle_name}"
        )

        self.get_logger().info(
            f"CARLA Simulation Asset: {self.carla_vehicle_asset}"
        )

        self.get_logger().info(
            "CINEMATIC REAR 3/4 CHASE VIEW ENABLED"
        )

        self.get_logger().info(
            "ADAS FRONT CAMERA INSET ENABLED"
        )

        if self.webhook_url:
            self.get_logger().info(
                "UiPath maintenance webhook: CONFIGURED"
            )
        else:
            self.get_logger().warning(
                "UiPath maintenance webhook: NOT CONFIGURED"
            )

        self.get_logger().info(
            "NO DRIVER TAKEOVER in proposed V2X-HEAL path"
        )

        self.get_logger().info("=" * 70)

    # ================================================================
    # PYGAME SETUP
    # ================================================================

    def _setup_pygame(self):

        try:
            pygame.init()

            self.screen = pygame.display.set_mode(
                (
                    self.display_width,
                    self.display_height
                )
            )

            pygame.display.set_caption(
                "TAV - Toyota ADAS Verification | V2X-HEAL"
            )

            self.font_large = pygame.font.SysFont(
                "Arial",
                30,
                bold=True
            )

            self.font_medium = pygame.font.SysFont(
                "Arial",
                22,
                bold=True
            )

            self.font_small = pygame.font.SysFont(
                "Arial",
                18
            )

            self.font_tiny = pygame.font.SysFont(
                "Arial",
                14,
                bold=True
            )

            self.pygame_enabled = True

        except Exception as exc:
            self.get_logger().warning(
                f"Pygame unavailable: {exc}"
            )
            self.pygame_enabled = False

    # ================================================================
    # CARLA CONNECTION
    # ================================================================

    def _connect_to_carla(self):

        self.get_logger().info(
            f"Connecting to CARLA at {self.host}:{self.port}"
        )

        self.client = carla.Client(
            self.host,
            self.port
        )

        self.client.set_timeout(30.0)

        self.world = self.client.get_world()

        current_map = self.world.get_map().name

        if not current_map.endswith(self.map_name):

            self.get_logger().info(
                f"Current map: {current_map}. "
                f"Loading {self.map_name}..."
            )

            self.world = self.client.load_world(
                self.map_name
            )

            time.sleep(3.0)

        self.carla_map = self.world.get_map()

        self.get_logger().info(
            f"CARLA BACKEND CONNECTED | "
            f"Map: {self.carla_map.name}"
        )

        self._spawn_vehicle()
        self._spawn_front_camera()
        self._spawn_chase_camera()
        self._start_driving()
        self._update_spectator()

    # ================================================================
    # VEHICLE
    # ================================================================

    def _spawn_vehicle(self):

        blueprint_library = (
            self.world.get_blueprint_library()
        )

        candidates = blueprint_library.filter(
            "vehicle.toyota.prius"
        )

        if not candidates:

            candidates = blueprint_library.filter(
                "vehicle.tesla.model3"
            )

            self.carla_vehicle_asset = (
                "Tesla Model 3 "
                "(fallback simulation surrogate)"
            )

        if not candidates:

            raise RuntimeError(
                "No suitable CARLA vehicle found."
            )

        blueprint = candidates[0]

        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute(
                "role_name",
                "TAV_ADAS_Prototype"
            )

        # Try to choose a strong visible vehicle color.
        if blueprint.has_attribute("color"):

            try:
                colors = (
                    blueprint
                    .get_attribute("color")
                    .recommended_values
                )

                preferred = None

                for color in colors:

                    parts = color.split(",")

                    if len(parts) != 3:
                        continue

                    r = int(parts[0])
                    g = int(parts[1])
                    b = int(parts[2])

                    if r > 110 and r > g * 1.25 and r > b * 1.25:
                        preferred = color
                        break

                if preferred:
                    blueprint.set_attribute(
                        "color",
                        preferred
                    )

            except Exception:
                pass

        spawn_points = (
            self.carla_map.get_spawn_points()
        )

        if not spawn_points:
            raise RuntimeError(
                "No spawn points available."
            )

        random.shuffle(spawn_points)

        for spawn_point in spawn_points:

            vehicle = self.world.try_spawn_actor(
                blueprint,
                spawn_point
            )

            if vehicle is not None:
                self.vehicle = vehicle
                break

        if self.vehicle is None:
            raise RuntimeError(
                "Unable to spawn TAV vehicle."
            )

        self.get_logger().info(
            f"Vehicle spawned | Actor ID={self.vehicle.id}"
        )

    # ================================================================
    # FRONT ADAS CAMERA
    # ================================================================

    def _spawn_front_camera(self):

        camera_bp = (
            self.world
            .get_blueprint_library()
            .find("sensor.camera.rgb")
        )

        camera_bp.set_attribute(
            "image_size_x",
            str(self.camera_width)
        )

        camera_bp.set_attribute(
            "image_size_y",
            str(self.camera_height)
        )

        camera_bp.set_attribute(
            "fov",
            "90"
        )

        camera_bp.set_attribute(
            "sensor_tick",
            "0.05"
        )

        front_transform = carla.Transform(
            carla.Location(
                x=1.8,
                y=0.0,
                z=1.55
            ),
            carla.Rotation(
                pitch=-2.0,
                yaw=0.0,
                roll=0.0
            )
        )

        self.front_camera = self.world.spawn_actor(
            camera_bp,
            front_transform,
            attach_to=self.vehicle,
            attachment_type=carla.AttachmentType.Rigid
        )

        self.front_camera.listen(
            self._front_camera_callback
        )

        self.get_logger().info(
            "ADAS front camera initialized."
        )

    # ================================================================
    # CINEMATIC CHASE CAMERA
    # ================================================================

    def _spawn_chase_camera(self):

        camera_bp = (
            self.world
            .get_blueprint_library()
            .find("sensor.camera.rgb")
        )

        camera_bp.set_attribute(
            "image_size_x",
            str(self.camera_width)
        )

        camera_bp.set_attribute(
            "image_size_y",
            str(self.camera_height)
        )

        # Lower FOV makes the vehicle look larger
        # and gives a more cinematic perspective.
        camera_bp.set_attribute(
            "fov",
            "82"
        )

        camera_bp.set_attribute(
            "sensor_tick",
            "0.05"
        )

        # ============================================================
        # MAIN CAMERA POSITION
        #
        # x = behind car
        # y = slightly towards right side
        # z = moderately above car
        #
        # IMPORTANT:
        # This is NOT bird's-eye.
        # It gives a rear 3/4 vehicle view.
        # ============================================================

        chase_transform = carla.Transform(
            carla.Location(
                x=-7.5,
                y=2.3,
                z=3.0
            ),
            carla.Rotation(
                pitch=-10.0,
                yaw=-13.0,
                roll=0.0
            )
        )

        try:

            self.chase_camera = self.world.spawn_actor(
                camera_bp,
                chase_transform,
                attach_to=self.vehicle,
                attachment_type=(
                    carla.AttachmentType.SpringArmGhost
                )
            )

        except Exception:

            self.chase_camera = self.world.spawn_actor(
                camera_bp,
                chase_transform,
                attach_to=self.vehicle,
                attachment_type=(
                    carla.AttachmentType.Rigid
                )
            )

        self.chase_camera.listen(
            self._chase_camera_callback
        )

        self.get_logger().info(
            "Cinematic rear 3/4 chase camera initialized."
        )

    # ================================================================
    # CARLA IMAGE CONVERSION
    # ================================================================

    def _image_to_rgb_array(self, image):

        array = np.frombuffer(
            image.raw_data,
            dtype=np.uint8
        )

        array = array.reshape(
            (
                image.height,
                image.width,
                4
            )
        )

        # Remove alpha.
        array = array[:, :, :3]

        # BGR -> RGB.
        array = array[:, :, ::-1]

        return array.copy()

    # ================================================================
    # CAMERA CALLBACKS
    # ================================================================

    def _front_camera_callback(self, image):

        self.front_frame = (
            self._image_to_rgb_array(image)
        )

    def _chase_camera_callback(self, image):

        self.chase_frame = (
            self._image_to_rgb_array(image)
        )

    # ================================================================
    # ADAS CAMERA YAW
    # ================================================================

    def _set_front_camera_yaw(self, yaw_deg):

        if self.front_camera is None:
            return

        transform = carla.Transform(
            carla.Location(
                x=1.8,
                y=0.0,
                z=1.55
            ),
            carla.Rotation(
                pitch=-2.0,
                yaw=float(yaw_deg),
                roll=0.0
            )
        )

        self.front_camera.set_transform(
            transform
        )

    # ================================================================
    # AUTONOMOUS DRIVING
    # ================================================================

    def _start_driving(self):

        try:

            self.traffic_manager = (
                self.client.get_trafficmanager(8000)
            )

            self.traffic_manager.set_synchronous_mode(
                False
            )

            self.traffic_manager.auto_lane_change(
                self.vehicle,
                False
            )

            # Make demo speed reasonable.
            self.traffic_manager.vehicle_percentage_speed_difference(
                self.vehicle,
                35.0
            )

            self.vehicle.set_autopilot(
                True,
                8000
            )

            self.autopilot_enabled = True

            self.get_logger().info(
                "Traffic Manager autonomous driving enabled."
            )

        except Exception as exc:

            self.autopilot_enabled = False

            self.get_logger().warning(
                f"Traffic Manager unavailable: {exc}"
            )

            self.get_logger().warning(
                "Fallback lane follower enabled."
            )

    # ================================================================
    # FALLBACK CONTROLLER
    # ================================================================

    def _fallback_drive(self):

        if self.vehicle is None:
            return

        if self.autopilot_enabled:
            return

        vehicle_transform = (
            self.vehicle.get_transform()
        )

        waypoint = self.carla_map.get_waypoint(
            vehicle_transform.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )

        if waypoint is None:
            return

        next_waypoints = waypoint.next(
            8.0
        )

        if not next_waypoints:
            return

        target = (
            next_waypoints[0]
            .transform
            .location
        )

        dx = (
            target.x
            - vehicle_transform.location.x
        )

        dy = (
            target.y
            - vehicle_transform.location.y
        )

        target_yaw = math.atan2(
            dy,
            dx
        )

        current_yaw = math.radians(
            vehicle_transform.rotation.yaw
        )

        yaw_error = (
            target_yaw
            - current_yaw
        )

        while yaw_error > math.pi:
            yaw_error -= 2.0 * math.pi

        while yaw_error < -math.pi:
            yaw_error += 2.0 * math.pi

        steer = max(
            -1.0,
            min(
                1.0,
                yaw_error / math.radians(35.0)
            )
        )

        speed = self._get_vehicle_speed()

        throttle = 0.42

        if speed > 35.0:
            throttle = 0.10

        self.vehicle.apply_control(
            carla.VehicleControl(
                throttle=throttle,
                steer=steer,
                brake=0.0
            )
        )

    # ================================================================
    # CARLA MAIN SPECTATOR
    # ================================================================

    def _update_spectator(self):

        if self.world is None or self.vehicle is None:
            return

        try:

            spectator = (
                self.world.get_spectator()
            )

            vehicle_transform = (
                self.vehicle.get_transform()
            )

            yaw = math.radians(
                vehicle_transform.rotation.yaw
            )

            forward_x = math.cos(yaw)
            forward_y = math.sin(yaw)

            right_x = -math.sin(yaw)
            right_y = math.cos(yaw)

            rear_distance = 8.0
            side_offset = 2.5
            camera_height = 3.5

            camera_x = (
                vehicle_transform.location.x
                - rear_distance * forward_x
                + side_offset * right_x
            )

            camera_y = (
                vehicle_transform.location.y
                - rear_distance * forward_y
                + side_offset * right_y
            )

            camera_z = (
                vehicle_transform.location.z
                + camera_height
            )

            spectator_transform = carla.Transform(
                carla.Location(
                    x=camera_x,
                    y=camera_y,
                    z=camera_z
                ),
                carla.Rotation(
                    pitch=-12.0,
                    yaw=(
                        vehicle_transform.rotation.yaw
                        - 14.0
                    ),
                    roll=0.0
                )
            )

            spectator.set_transform(
                spectator_transform
            )

        except Exception:
            pass

    # ================================================================
    # ROS CALLBACKS
    # ================================================================

    def fault_state_callback(self, msg):

        self.fault_active = bool(
            msg.data
        )

    def fault_yaw_callback(self, msg):

        self.latest_fault_yaw = float(
            msg.data
        )

    # ================================================================
    # SEVERITY
    # ================================================================

    def _get_fault_severity(self):

        yaw = abs(
            self.raw_camera_yaw_deg
        )

        if yaw <= 4.0:
            return "MODERATE"

        if yaw <= 8.0:
            return "HIGH"

        return "CRITICAL"

    # ================================================================
    # MAINTENANCE MESSAGE
    # ================================================================

    def _get_maintenance_action(self):

        severity = (
            self._get_fault_severity()
        )

        if severity == "MODERATE":

            return (
                "Schedule ADAS camera inspection and "
                "calibration verification."
            )

        if severity == "HIGH":

            return (
                "ADAS camera mounting inspection and "
                "recalibration are recommended before "
                "extended vehicle operation."
            )

        return (
            "High-priority camera mounting inspection "
            "and full ADAS recalibration are recommended."
        )

    # ================================================================
    # SPEED
    # ================================================================

    def _get_vehicle_speed(self):

        if self.vehicle is None:
            return 0.0

        try:

            velocity = (
                self.vehicle.get_velocity()
            )

            speed = math.sqrt(
                velocity.x ** 2
                + velocity.y ** 2
                + velocity.z ** 2
            )

            return speed * 3.6

        except Exception:
            return 0.0

    # ================================================================
    # LANE OFFSET
    # ================================================================

    def _get_lane_offset(self):

        if self.vehicle is None:
            return 0.0

        try:

            vehicle_location = (
                self.vehicle.get_location()
            )

            waypoint = self.carla_map.get_waypoint(
                vehicle_location,
                project_to_road=True,
                lane_type=carla.LaneType.Driving
            )

            if waypoint is None:
                return 0.0

            lane_center = (
                waypoint.transform.location
            )

            right_vector = (
                waypoint
                .transform
                .get_right_vector()
            )

            dx = (
                vehicle_location.x
                - lane_center.x
            )

            dy = (
                vehicle_location.y
                - lane_center.y
            )

            offset = (
                dx * right_vector.x
                + dy * right_vector.y
            )

            return float(offset)

        except Exception:
            return 0.0

    # ================================================================
    # MAIN DEMO
    # ================================================================

    def update_demo(self):

        if self.pygame_enabled:

            for event in pygame.event.get():

                if event.type == pygame.QUIT:
                    self.done = True
                    return

        self._fallback_drive()
        self._update_spectator()

        elapsed_demo = (
            time.monotonic()
            - self.demo_start_time
        )

        # ============================================================
        # DETECT FAULT
        # ============================================================

        if (
            self.fault_active
            and not self.fault_detected
            and abs(self.latest_fault_yaw) > 0.1
        ):

            self.fault_detected = True

            self.fault_start_time = (
                time.monotonic()
            )

            self.raw_camera_yaw_deg = (
                self.latest_fault_yaw
            )

            self.software_correction_deg = 0.0

            self.effective_yaw_deg = (
                self.raw_camera_yaw_deg
            )

            self.phase = "FAULT DETECTED"

            self.status_text = (
                "CAMERA CALIBRATION DRIFT DETECTED"
            )

            self._set_front_camera_yaw(
                self.raw_camera_yaw_deg
            )

            self.get_logger().warning(
                f"PHASE B: RAW CAMERA YAW "
                f"{self.raw_camera_yaw_deg:+.1f} deg"
            )

            self.get_logger().warning(
                "FAIL-OPERATIONAL HOLD ACTIVE | "
                "AUTONOMY MAINTAINED"
            )

        # ============================================================
        # RECOVERY
        # ============================================================

        if (
            self.fault_detected
            and not self.recovery_complete
        ):

            elapsed_fault = (
                time.monotonic()
                - self.fault_start_time
            )

            # --------------------------------------------------------
            # FAULT HOLD
            # --------------------------------------------------------

            if elapsed_fault < self.fault_hold_sec:

                self.phase = "FAULT DETECTED"

                self.status_text = (
                    "CAMERA CALIBRATION DRIFT DETECTED"
                )

                self.software_correction_deg = 0.0

                self.effective_yaw_deg = (
                    self.raw_camera_yaw_deg
                )

            # --------------------------------------------------------
            # SELF-CALIBRATION
            # --------------------------------------------------------

            else:

                correction_elapsed = (
                    elapsed_fault
                    - self.fault_hold_sec
                )

                progress = min(
                    1.0,
                    correction_elapsed
                    / self.calibration_duration_sec
                )

                self.phase = "SELF-CALIBRATING"

                self.status_text = (
                    "V2X-HEAL SELF-CALIBRATION ACTIVE"
                )

                self.software_correction_deg = (
                    -self.raw_camera_yaw_deg
                    * progress
                )

                self.effective_yaw_deg = (
                    self.raw_camera_yaw_deg
                    + self.software_correction_deg
                )

                self._set_front_camera_yaw(
                    self.effective_yaw_deg
                )

                # ----------------------------------------------------
                # RECOVERED
                # ----------------------------------------------------

                if progress >= 1.0:

                    self.recovery_complete = True

                    self.phase = "RECOVERED"

                    self.status_text = (
                        "FAIL-OPERATIONAL RECOVERY COMPLETE"
                    )

                    self.software_correction_deg = (
                        -self.raw_camera_yaw_deg
                    )

                    self.effective_yaw_deg = 0.0

                    self._set_front_camera_yaw(
                        0.0
                    )

                    self.get_logger().info(
                        "PHASE C: V2X-HEAL "
                        "SELF-CALIBRATION COMPLETE"
                    )

                    self.get_logger().info(
                        "RECOVERED | "
                        f"RAW {self.raw_camera_yaw_deg:+.1f} | "
                        f"CORR "
                        f"{self.software_correction_deg:+.1f} | "
                        "EFFECTIVE 0.0 | "
                        "NO TAKEOVER"
                    )

                    self._trigger_maintenance_alert()

        self._render_dashboard(
            elapsed_demo
        )

        # ============================================================
        # END
        # ============================================================

        if elapsed_demo >= self.demo_duration_sec:

            self.get_logger().info(
                "60 SECOND FAIL-OPERATIONAL "
                "V2X-HEAL DEMO COMPLETE"
            )

            self.done = True

    # ================================================================
    # CONVERT FRAME TO PYGAME
    # ================================================================

    def _frame_to_surface(self, frame):

        if frame is None:
            return None

        display_array = np.swapaxes(
            frame,
            0,
            1
        )

        return pygame.surfarray.make_surface(
            display_array
        )

    # ================================================================
    # DASHBOARD
    # ================================================================

    def _render_dashboard(self, elapsed):

        if not self.pygame_enabled:
            return

        self.screen.fill(
            (18, 18, 18)
        )

        # ============================================================
        # MAIN VEHICLE VIEW
        # ============================================================

        chase_surface = (
            self._frame_to_surface(
                self.chase_frame
            )
        )

        if chase_surface is not None:

            chase_surface = (
                pygame.transform.smoothscale(
                    chase_surface,
                    (
                        self.display_width,
                        self.main_view_height
                    )
                )
            )

            self.screen.blit(
                chase_surface,
                (0, 0)
            )

        # ============================================================
        # MAIN VIEW TITLE
        # ============================================================

        title_overlay = pygame.Surface(
            (390, 48),
            pygame.SRCALPHA
        )

        title_overlay.fill(
            (0, 0, 0, 175)
        )

        self.screen.blit(
            title_overlay,
            (20, 20)
        )

        chase_title = self.font_small.render(
            "VEHICLE BEHAVIOR / CINEMATIC CHASE VIEW",
            True,
            (255, 255, 255)
        )

        self.screen.blit(
            chase_title,
            (32, 34)
        )

        # ============================================================
        # ADAS CAMERA INSET
        # ============================================================

        front_surface = (
            self._frame_to_surface(
                self.front_frame
            )
        )

        inset_width = 360
        inset_height = 203

        inset_x = (
            self.display_width
            - inset_width
            - 28
        )

        inset_y = 28

        if front_surface is not None:

            front_surface = (
                pygame.transform.smoothscale(
                    front_surface,
                    (
                        inset_width,
                        inset_height
                    )
                )
            )

            self.screen.blit(
                front_surface,
                (
                    inset_x,
                    inset_y
                )
            )

            # White border
            pygame.draw.rect(
                self.screen,
                (255, 255, 255),
                (
                    inset_x - 3,
                    inset_y - 3,
                    inset_width + 6,
                    inset_height + 6
                ),
                3
            )

            # Bottom label
            pygame.draw.rect(
                self.screen,
                (0, 0, 0),
                (
                    inset_x,
                    inset_y + inset_height - 31,
                    inset_width,
                    31
                )
            )

            adas_text = self.font_tiny.render(
                "ADAS FRONT CAMERA",
                True,
                (255, 255, 255)
            )

            self.screen.blit(
                adas_text,
                (
                    inset_x + 12,
                    inset_y + inset_height - 23
                )
            )

            # ========================================================
            # YAW STATUS COLOR
            # ========================================================

            if self.phase == "NORMAL":

                yaw_color = (
                    40,
                    150,
                    75
                )

            elif self.phase == "FAULT DETECTED":

                yaw_color = (
                    205,
                    45,
                    45
                )

            elif self.phase == "SELF-CALIBRATING":

                yaw_color = (
                    225,
                    140,
                    20
                )

            else:

                yaw_color = (
                    35,
                    135,
                    210
                )

            pygame.draw.rect(
                self.screen,
                yaw_color,
                (
                    inset_x + 10,
                    inset_y + 10,
                    175,
                    32
                )
            )

            yaw_text = self.font_tiny.render(
                f"Yaw: "
                f"{self.effective_yaw_deg:+.1f} deg",
                True,
                (255, 255, 255)
            )

            self.screen.blit(
                yaw_text,
                (
                    inset_x + 18,
                    inset_y + 18
                )
            )

        # ============================================================
        # BOTTOM STATUS PANEL
        # ============================================================

        panel_y = (
            self.main_view_height
        )

        pygame.draw.rect(
            self.screen,
            (18, 18, 18),
            (
                0,
                panel_y,
                self.display_width,
                self.status_panel_height
            )
        )

        # Status banner
        if self.phase == "NORMAL":

            status_color = (
                40,
                155,
                75
            )

        elif self.phase == "FAULT DETECTED":

            status_color = (
                205,
                45,
                45
            )

        elif self.phase == "SELF-CALIBRATING":

            status_color = (
                225,
                140,
                20
            )

        else:

            status_color = (
                35,
                135,
                210
            )

        pygame.draw.rect(
            self.screen,
            status_color,
            (
                0,
                panel_y,
                self.display_width,
                50
            )
        )

        status_title = self.font_medium.render(
            f"TAV | {self.status_text}",
            True,
            (255, 255, 255)
        )

        self.screen.blit(
            status_title,
            (
                20,
                panel_y + 13
            )
        )

        # ============================================================
        # TELEMETRY
        # ============================================================

        speed = (
            self._get_vehicle_speed()
        )

        lane_offset = (
            self._get_lane_offset()
        )

        severity = (
            self._get_fault_severity()
            if self.fault_detected
            else "NONE"
        )

        line1 = self.font_small.render(
            f"Vehicle: {self.vehicle_name}     "
            f"Run ID: {self.tav_run_id}     "
            f"Speed: {speed:.1f} km/h",
            True,
            (240, 240, 240)
        )

        line2 = self.font_small.render(
            f"Raw Camera Yaw: "
            f"{self.raw_camera_yaw_deg:+.1f} deg     "
            f"Software Correction: "
            f"{self.software_correction_deg:+.1f} deg     "
            f"Effective Yaw: "
            f"{self.effective_yaw_deg:+.1f} deg",
            True,
            (240, 240, 240)
        )

        line3 = self.font_small.render(
            f"Autonomy: MAINTAINED     "
            f"Driver Takeover: NO     "
            f"Lane Offset: {lane_offset:+.2f} m     "
            f"Severity: {severity}     "
            f"Time: {elapsed:05.1f}s",
            True,
            (240, 240, 240)
        )

        self.screen.blit(
            line1,
            (
                20,
                panel_y + 66
            )
        )

        self.screen.blit(
            line2,
            (
                20,
                panel_y + 105
            )
        )

        self.screen.blit(
            line3,
            (
                20,
                panel_y + 143
            )
        )

        pygame.display.flip()

    # ================================================================
    # UIPATH TRIGGER
    # ================================================================

    def _trigger_maintenance_alert(self):

        if self.webhook_sent:
            return

        self.webhook_sent = True

        if not self.webhook_url:

            self.get_logger().warning(
                "Recovery completed, but "
                "UIPATH_WEBHOOK_URL is not set. "
                "Maintenance email skipped."
            )

            return

        thread = threading.Thread(
            target=self._send_webhook,
            daemon=True
        )

        thread.start()

    # ================================================================
    # UIPATH EMAIL
    # ================================================================

    def _send_webhook(self):

        severity = (
            self._get_fault_severity()
        )

        maintenance_action = (
            self._get_maintenance_action()
        )

        lane_offset = (
            self._get_lane_offset()
        )

        timestamp_utc = (
            datetime.datetime.now(
                datetime.timezone.utc
            ).isoformat()
        )

        email_subject = (
            "TAV ADAS Health Alert | "
            "Camera Calibration Drift | "
            f"{severity}"
        )

        email_body = f"""
<div style="
font-family:Arial,Helvetica,sans-serif;
background:#f4f4f4;
padding:24px;
">

<div style="
max-width:760px;
margin:auto;
background:#ffffff;
border-top:6px solid #e50000;
">

<div style="padding:24px 30px;">

<table style="
width:100%;
border-collapse:collapse;
">

<tr>

<td style="
width:175px;
vertical-align:middle;
">

<img
src="{self.logo_url}"
alt="TAV Logo"
style="
width:150px;
max-height:85px;
object-fit:contain;
"
/>

</td>

<td style="vertical-align:middle;">

<div style="
font-size:28px;
font-weight:700;
color:#222222;
">
TAV
</div>

<div style="
font-size:16px;
color:#444444;
">
Toyota ADAS Verification
</div>

<div style="
font-size:12px;
color:#888888;
margin-top:5px;
">
Project Prototype |
Automated ADAS Health Monitoring
</div>

</td>

</tr>

</table>

</div>


<div style="
padding:22px 30px;
background:#fafafa;
border-top:1px solid #eeeeee;
border-bottom:1px solid #eeeeee;
">

<div style="
font-size:22px;
font-weight:700;
color:#222222;
">
Camera Calibration Drift Detected
</div>

<div style="
display:inline-block;
margin-top:12px;
background:#e85d04;
color:white;
font-weight:700;
padding:8px 14px;
border-radius:20px;
">
{severity} SEVERITY
</div>

</div>


<div style="padding:26px 30px;">

<div style="
color:#e50000;
font-weight:700;
margin-bottom:16px;
">
VEHICLE INFORMATION
</div>

<table style="
width:100%;
line-height:2;
border-collapse:collapse;
">

<tr>
<td style="color:#666;">
Vehicle
</td>

<td>
<b>{self.vehicle_name}</b>
</td>
</tr>


<tr>
<td style="color:#666;">
Vehicle ID / Demo VIN
</td>

<td>
<b>{self.demo_vin}</b>
</td>
</tr>


<tr>
<td style="color:#666;">
CARLA Simulation Asset
</td>

<td>
{self.carla_vehicle_asset}
</td>
</tr>


<tr>
<td style="color:#666;">
Simulation Map
</td>

<td>
{self.carla_map.name.split('/')[-1]}
</td>
</tr>


<tr>
<td style="color:#666;">
TAV Run ID
</td>

<td>
<b>{self.tav_run_id}</b>
</td>
</tr>


<tr>
<td style="color:#666;">
System Mode
</td>

<td>
<b>FAIL-OPERATIONAL</b>
</td>
</tr>

</table>


<hr style="
border:none;
border-top:1px solid #eeeeee;
margin:24px 0;
">


<div style="
color:#e50000;
font-weight:700;
margin-bottom:16px;
">
DETECTED CONDITION
</div>

<table style="
width:100%;
line-height:2;
border-collapse:collapse;
">

<tr>
<td style="color:#666;">
Raw Camera Yaw Drift
</td>

<td>
<b>{self.raw_camera_yaw_deg:+.1f} degrees</b>
</td>
</tr>


<tr>
<td style="color:#666;">
Software Compensation
</td>

<td>
<b>{self.software_correction_deg:+.1f} degrees</b>
</td>
</tr>


<tr>
<td style="color:#666;">
Effective Yaw After Recovery
</td>

<td>
<b>0.0 degrees</b>
</td>
</tr>


<tr>
<td style="color:#666;">
Lane Offset
</td>

<td>
{lane_offset:+.3f} m
</td>
</tr>

</table>


<hr style="
border:none;
border-top:1px solid #eeeeee;
margin:24px 0;
">


<div style="
color:#e50000;
font-weight:700;
margin-bottom:16px;
">
SYSTEM RESPONSE
</div>

<p style="line-height:1.8;">

<b>Autonomy maintained:</b>
YES

<br>

<b>Driver takeover required:</b>
NO

<br>

<b>V2X-HEAL software recovery:</b>
SUCCESSFUL

</p>


<div style="
margin-top:20px;
padding:16px;
background:#fff3f3;
border-left:4px solid #e50000;
line-height:1.6;
">

<b>Service Recommendation</b>

<br><br>

{maintenance_action}

</div>


<p style="
margin-top:22px;
color:#555555;
line-height:1.6;
">

The V2X-HEAL prototype detected a simulated ADAS
camera calibration drift and applied software compensation
while vehicle operation remained active.

Physical camera inspection and recalibration remain
recommended after the recovery event.

</p>


<div style="
margin-top:25px;
font-size:12px;
color:#999999;
line-height:1.6;
">

Timestamp UTC:
{timestamp_utc}

<br>

TAV is an academic/startup project prototype.
It is not an official Toyota production system.

</div>

</div>

</div>

</div>
"""

        payload = {
            "event": "V2X_HEAL_RECOVERY",

            "source":
                "TAV - Toyota ADAS Verification",

            "prototype_notice":
                "Academic / startup concept prototype",

            "tav_run_id":
                self.tav_run_id,

            "timestamp_utc":
                timestamp_utc,

            "vehicle":
                self.vehicle_name,

            "demo_vin":
                self.demo_vin,

            "carla_vehicle_asset":
                self.carla_vehicle_asset,

            "map":
                self.carla_map.name.split("/")[-1],

            "system_mode":
                "FAIL_OPERATIONAL",

            "raw_camera_yaw_deg":
                round(
                    float(self.raw_camera_yaw_deg),
                    2
                ),

            "software_correction_deg":
                round(
                    float(self.software_correction_deg),
                    2
                ),

            "effective_yaw_deg":
                0.0,

            "fault_severity":
                severity,

            "actual_lane_offset_m":
                round(
                    float(lane_offset),
                    3
                ),

            "driver_takeover":
                False,

            "autonomy_maintained":
                True,

            "maintenance_required":
                True,

            "recommended_action":
                maintenance_action,

            "email_subject":
                email_subject,

            "email_body":
                email_body
        }

        try:

            request_body = json.dumps(
                payload
            ).encode("utf-8")

            request = urllib.request.Request(
                self.webhook_url,
                data=request_body,
                headers={
                    "Content-Type":
                        "application/json"
                },
                method="POST"
            )

            with urllib.request.urlopen(
                request,
                timeout=15
            ) as response:

                response_text = (
                    response
                    .read()
                    .decode(
                        "utf-8",
                        errors="ignore"
                    )
                )

                self.get_logger().info(
                    "UiPath maintenance webhook accepted "
                    f"(HTTP {response.status})"
                )

                if response_text:

                    self.get_logger().info(
                        f"UiPath response: {response_text}"
                    )

        except Exception as exc:

            self.get_logger().error(
                f"UiPath webhook failed: {exc}"
            )

    # ================================================================
    # CLEANUP
    # ================================================================

    def cleanup(self):

        self.get_logger().info(
            "Cleaning up TAV CARLA simulation..."
        )

        if self.front_camera is not None:

            try:
                self.front_camera.stop()
            except Exception:
                pass

            try:
                self.front_camera.destroy()
            except Exception:
                pass

            self.front_camera = None

        if self.chase_camera is not None:

            try:
                self.chase_camera.stop()
            except Exception:
                pass

            try:
                self.chase_camera.destroy()
            except Exception:
                pass

            self.chase_camera = None

        if self.vehicle is not None:

            try:
                self.vehicle.set_autopilot(False)
            except Exception:
                pass

            try:
                self.vehicle.destroy()
            except Exception:
                pass

            self.vehicle = None

        if self.pygame_enabled:

            try:
                pygame.quit()
            except Exception:
                pass


# ================================================================
# MAIN
# ================================================================

def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = V2XHealCarlaVisualizer()

        while (
            rclpy.ok()
            and not node.done
        ):

            rclpy.spin_once(
                node,
                timeout_sec=0.05
            )

    except KeyboardInterrupt:
        pass

    except Exception as exc:

        print(
            f"V2X-HEAL ERROR: {exc}"
        )

        raise

    finally:

        if node is not None:

            try:
                node.cleanup()
            except Exception:
                pass

            try:
                node.destroy_node()
            except Exception:
                pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()