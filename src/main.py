# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       someon                                                       #
# 	Created:      11/11/2025,                                                  #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import Brain, Ports, Motor, GearSetting, Controller, PRIMARY, Rotation, DEGREES, MSEC, SEC, wait, Thread, FORWARD, REVERSE, PERCENT, BRAKE, COAST, Timer, Competition
import math
import os

# Push Back 2025-26 (official manual available at https://www.vexrobotics.com/push-back-manual)
# Key timing constraints (Head-to-Head): Autonomous: 15s, Driver Controlled: 1:45.
# Endgame: last 20s (SG11: Park Zones protected). Autonomous Win Point (SC6) requires
# scoring thresholds; this sample auton is a simplified path follower and does NOT
# attempt full AWP criteria yet. Tune ports, directions, timing constants below on-field.

# Brain (display / timer)
brain = Brain()

# --- Tunables for time-based following (calibrate on your robot) ---
# Approx milliseconds per inch when driving straight at DRIVE_SPEED_PCT
MS_PER_INCH = 15.0  # smaller = faster; tune on the field
# Approx milliseconds per degree when turning in place at TURN_SPEED_PCT
MS_PER_DEGREE = 3.0  # smaller = faster; tune on the field

# Match phase durations (ms)
AUTON_DURATION_MS = 15_000
DRIVER_DURATION_MS = 105_000  # 1:45
ENDGAME_WARN_MS = DRIVER_DURATION_MS - 20_000  # last 20s start

# Default speeds used by the follower
DRIVE_SPEED_PCT = 65
TURN_SPEED_PCT = 55

# Downsample the dense path to every Nth point (reduces micro turns)
PATH_STRIDE = 5

# Select which of the 4 routes in path.txt to run (0..3)
# The order corresponds to the blocks in the file (each begins with '#PATH-POINTS-START Path').
START_PATH_INDEX = 0

# --- Odometry (tracking wheels + rotation sensors) configuration ---
# Toggle odometry-based autonomous (True) or keep existing time-based follower (False).
USE_ODOMETRY = True

# Rotation sensor ports for tracking pods (adjust to your wiring)
ODOM_LEFT_PORT = Ports.PORT5
ODOM_RIGHT_PORT = Ports.PORT6
ODOM_CENTER_PORT = Ports.PORT7

# If a sensor is mounted reversed, flip here
ODOM_LEFT_REVERSED = False
ODOM_RIGHT_REVERSED = True
ODOM_CENTER_REVERSED = False

# Physical constants (inches)
# Tracking wheel diameter (2.75" is common for VEX odom wheels)
ODOM_WHEEL_DIAM_IN = 2.75
# Distance between left and right tracking wheels (center-to-center)
ODOM_TRACK_WIDTH_IN = 10.5
# Lateral wheel offset from the robot rotation center along the forward axis (positive forward)
ODOM_CENTER_OFFSET_IN = 0.0

# Simple proportional gains for the odometry follower
ODOM_LIN_GAIN = 8.0   # pct per inch (capped by DRIVE_SPEED_PCT)
ODOM_TURN_GAIN_DEG = 0.8  # pct per degree (capped by TURN_SPEED_PCT)
ODOM_DIST_TOL_IN = 0.75
ODOM_HEAD_TOL_DEG = 5.0

# --- Robot configuration (update ports if your wiring is different) ---
# Left and right drive motors (flip the 'reverse' boolean if a motor spins
# the wrong direction when commanded forward)
left_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
right_motor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)

# Simple intake / manipulator motor
intake = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)

# Optional lift or indexer (unused in this generic example)
lift = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)

# Controller
controller = Controller(PRIMARY)

# Rotation sensors for odometry tracking wheels
left_track = Rotation(ODOM_LEFT_PORT, ODOM_LEFT_REVERSED)
right_track = Rotation(ODOM_RIGHT_PORT, ODOM_RIGHT_REVERSED)
center_track = Rotation(ODOM_CENTER_PORT, ODOM_CENTER_REVERSED)


# --- Odometry implementation ---
class Odometry:
	def __init__(self,
				 left: Rotation,
				 right: Rotation,
				 center: Rotation,
				 wheel_diam_in: float,
				 track_width_in: float,
				 center_offset_in: float) -> None:
		self.left = left
		self.right = right
		self.center = center
		self.wheel_circ_in = math.pi * float(wheel_diam_in)
		self.track_width_in = float(track_width_in)
		self.center_offset_in = float(center_offset_in)

		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0  # radians

		self._prev_deg_l = 0.0
		self._prev_deg_r = 0.0
		self._prev_deg_c = 0.0

	def reset(self, x_in: float = 0.0, y_in: float = 0.0, theta_deg: float = 0.0) -> None:
		"""Zero sensors and set pose (x,y in inches, heading in degrees)."""
		try:
			self.left.reset_position()
			self.right.reset_position()
			self.center.reset_position()
		except Exception:
			# Older firmware: fall back to set_position
			try:
				self.left.set_position(0, DEGREES)
				self.right.set_position(0, DEGREES)
				self.center.set_position(0, DEGREES)
			except Exception:
				pass

		self._prev_deg_l = 0.0
		self._prev_deg_r = 0.0
		self._prev_deg_c = 0.0
		self.x = float(x_in)
		self.y = float(y_in)
		self.theta = math.radians(float(theta_deg))

	def _deg_to_inches(self, deg: float) -> float:
		return (deg / 360.0) * self.wheel_circ_in

	def update(self) -> None:
		"""Integrate one odometry step from rotation deltas."""
		try:
			deg_l = self.left.position(DEGREES)
			deg_r = self.right.position(DEGREES)
			deg_c = self.center.position(DEGREES)
		except Exception:
			# If sensor read fails, skip this update
			return

		ddeg_l = deg_l - self._prev_deg_l
		ddeg_r = deg_r - self._prev_deg_r
		ddeg_c = deg_c - self._prev_deg_c

		# Update prevs early to avoid double counting on reentry
		self._prev_deg_l = deg_l
		self._prev_deg_r = deg_r
		self._prev_deg_c = deg_c

		dl = self._deg_to_inches(ddeg_l)
		dr = self._deg_to_inches(ddeg_r)
		dc = self._deg_to_inches(ddeg_c)

		# Differential drive heading change
		dtheta = (dr - dl) / self.track_width_in  # radians if we treat distances as arc lengths over width

		# Forward and lateral local displacements
		df = 0.5 * (dl + dr)
		ds = dc - (self.center_offset_in * dtheta)

		half = 0.5 * dtheta
		s = math.sin(self.theta + half)
		c = math.cos(self.theta + half)

		dx = df * c - ds * s
		dy = df * s + ds * c

		self.x += dx
		self.y += dy
		self.theta += dtheta

	def pose(self):
		return (self.x, self.y, math.degrees(self.theta))


_odom = Odometry(left_track, right_track, center_track,
				 ODOM_WHEEL_DIAM_IN, ODOM_TRACK_WIDTH_IN, ODOM_CENTER_OFFSET_IN)
_odom_thread_started = False


def start_odometry_thread() -> None:
	global _odom_thread_started
	if _odom_thread_started:
		return

	def _task():
		while True:
			_odom.update()
			wait(10, MSEC)

	try:
		Thread(_task)
		_odom_thread_started = True
	except Exception:
		# If threads aren't available in this environment, we can call update() synchronously
		_odom_thread_started = False


# --- Helper drive / actuator functions (time-based) ---
def drive_time(ms: int, speed_pct: int = 50, forward: bool = True) -> None:
	"""Spin both drive motors for a fixed duration (ms).

	This time-based helper is simple and robust for initial testing. Replace
	with encoder-based moves when you have a calibrated drivetrain.
	"""
	direction = FORWARD if forward else REVERSE
	left_motor.spin(direction, speed_pct, PERCENT)
	right_motor.spin(direction, speed_pct, PERCENT)
	wait(ms, MSEC)
	left_motor.stop(BRAKE)
	right_motor.stop(BRAKE)


def turn_time(ms: int, speed_pct: int = 50, left: bool = True) -> None:
	"""Turn in place for a fixed duration (ms)."""
	if left:
		left_motor.spin(REVERSE, speed_pct, PERCENT)
		right_motor.spin(FORWARD, speed_pct, PERCENT)
	else:
		left_motor.spin(FORWARD, speed_pct, PERCENT)
		right_motor.spin(REVERSE, speed_pct, PERCENT)
	wait(ms, MSEC)
	left_motor.stop(BRAKE)
	right_motor.stop(BRAKE)


def operate_intake(on: bool = True, speed_pct: int = 80) -> None:
	"""Turn the intake on or off."""
	if on:
		intake.spin(FORWARD, speed_pct, PERCENT)
	else:
		intake.stop(COAST)


# --- Path loading & following utilities ---
def _normalize_angle_deg(deg: float) -> float:
	"""Wrap angle to [-180, 180]."""
	while deg > 180.0:
		deg -= 360.0
	while deg < -180.0:
		deg += 360.0
	return deg


def _try_open_first(paths):
	for p in paths:
		if not p:
			continue
		try:
			return open(p, "r")
		except Exception:
			pass
	return None


def load_all_paths_from_file() -> list:
	"""Parse src/path.txt exported by a path planner into 4 lists of (x,y) waypoints.

	Returns a list of paths; each path is a list of (x, y) floats. Lines like
	"#PATH-POINTS-START" begin a new path. Extra columns (e.g., speed or flags)
	are ignored. Duplicate consecutive points are dropped.
	"""
	# Try common locations; fall back to local dir on the Brain
	candidates = []
	try:
		here = os.path.dirname(__file__)
		candidates.extend([
			os.path.join(here, "path.txt"),
			os.path.join(here, "src", "path.txt"),
		])
	except Exception:
		# __file__ may not exist in some environments
		pass
	candidates.extend(["src/path.txt", "path.txt"])  # workspace or brain root

	f = _try_open_first(candidates)
	if f is None:
		brain.screen.print("path.txt not found")
		return []

	paths = []
	current = []
	try:
		for raw in f:
			line = raw.strip()
			if not line:
				continue
			if line.startswith("#PATH.JERRYIO-DATA"):
				# end of numeric point dumps
				break
			if line.startswith("#PATH-POINTS-START"):
				if current:
					paths.append(current)
				current = []
				continue
			if line.startswith("#"):
				continue
			parts = [p for p in line.split(',') if p]
			if len(parts) < 2:
				continue
			try:
				x = float(parts[0])
				y = float(parts[1])
				if not current or (abs(x - current[-1][0]) > 1e-6 or abs(y - current[-1][1]) > 1e-6):
					current.append((x, y))
			except Exception:
				# ignore malformed numeric rows
				pass
		if current:
			paths.append(current)
	finally:
		try:
			f.close()
		except Exception:
			pass

	return paths


def follow_path_time_based(points: list,
						   stride: int = PATH_STRIDE,
						   drive_pct: int = DRIVE_SPEED_PCT,
						   turn_pct: int = TURN_SPEED_PCT,
						   max_ms: int | None = None) -> None:
	"""Follow a list of (x,y) waypoints by turning to each segment's heading then driving its length.

	This uses time-based helpers (no encoders/gyro required) and two timing constants:
	MS_PER_DEGREE and MS_PER_INCH. Tune those, plus drive/turn speeds, on the field.
	"""
	if not points or len(points) < 2:
		return

	# Downsample path to reduce micro-turns; ensure endpoints included
	step = max(1, int(stride))
	pts = points[::step]
	if pts[0] != points[0]:
		pts.insert(0, points[0])
	if pts[-1] != points[-1]:
		pts.append(points[-1])

	# Initial heading from first segment vector
	x0, y0 = pts[0]
	x1, y1 = pts[1]
	heading = math.degrees(math.atan2(y1 - y0, x1 - x0))

	# Optionally start intake for early game piece
	operate_intake(True, max(60, drive_pct))

	segment_start = Timer()
	for i in range(1, len(pts)):
		# Abort if time budget exceeded (for autonomous 15s safety)
		if max_ms is not None and segment_start.time(MSEC) >= max_ms:
			break
		xa, ya = pts[i - 1]
		xb, yb = pts[i]
		dx = xb - xa
		dy = yb - ya
		seg_deg = math.degrees(math.atan2(dy, dx))
		dtheta = _normalize_angle_deg(seg_deg - heading)
		dist_in = math.hypot(dx, dy)

		# Turn toward next segment
		if abs(dtheta) > 1.0:  # ignore tiny adjustments
			turn_ms = int(abs(dtheta) * MS_PER_DEGREE)
			turn_time(turn_ms, turn_pct, left=(dtheta > 0))
			heading = seg_deg

		# Drive the segment length
		if dist_in > 0.5:  # ignore tiny hops
			drive_ms = int(dist_in * MS_PER_INCH)
			drive_time(drive_ms, drive_pct, forward=True)

	# Briefly outtake/stop intake at the end if desired
	operate_intake(False)


def follow_path_odometry(points: list,
						 drive_pct: int = DRIVE_SPEED_PCT,
						 turn_pct: int = TURN_SPEED_PCT,
						 dist_tol_in: float = ODOM_DIST_TOL_IN,
						 head_tol_deg: float = ODOM_HEAD_TOL_DEG,
						 max_ms: int | None = None) -> None:
	"""Follow waypoints using live odometry pose; simple P control for translation & heading.

	This assumes _odom is running. It will drive to each waypoint in order, applying proportional
	speeds based on remaining distance and heading error. When within distance AND heading tolerance,
	it advances to the next point.
	"""
	if not points or len(points) < 2:
		return

	# Transform path into robot-local frame so that start is (0,0,0)
	x0, y0 = points[0]
	x1, y1 = points[1]
	a0 = math.atan2(y1 - y0, x1 - x0)
	ca = math.cos(-a0)
	sa = math.sin(-a0)
	local_pts = []
	for (px, py) in points:
		dx0 = px - x0
		dy0 = py - y0
		ux = dx0 * ca - dy0 * sa
		uy = dx0 * sa + dy0 * ca
		local_pts.append((ux, uy))

	# Intake start (optional piece collection at beginning)
	operate_intake(True, max(60, drive_pct))

	start_t = Timer()
	idx = 0
	while idx < len(local_pts):
		if max_ms is not None and start_t.time(MSEC) >= max_ms:
			break

		# Current pose
		x, y, heading_deg = _odom.pose()
		tx, ty = local_pts[idx]
		dx = tx - x
		dy = ty - y
		dist = math.hypot(dx, dy)
		target_heading_deg = math.degrees(math.atan2(dy, dx)) if dist > 0.01 else heading_deg
		head_err = _normalize_angle_deg(target_heading_deg - heading_deg)

		if dist <= dist_tol_in and abs(head_err) <= head_tol_deg:
			idx += 1
			continue

		# Proportional controls
		lin_cmd = dist * ODOM_LIN_GAIN  # pct
		turn_cmd = head_err * ODOM_TURN_GAIN_DEG  # pct

		# Cap commands
		lin_cmd = _cap(lin_cmd, -drive_pct, drive_pct)
		turn_cmd = _cap(turn_cmd, -turn_pct, turn_pct)

		# Convert to left/right motor velocities
		left_cmd = _cap(lin_cmd + turn_cmd)
		right_cmd = _cap(lin_cmd - turn_cmd)

		left_motor.set_velocity(left_cmd, PERCENT)
		right_motor.set_velocity(right_cmd, PERCENT)
		left_motor.spin(FORWARD)
		right_motor.spin(FORWARD)

		wait(20, MSEC)

	operate_intake(False)


# --- Autonomous routine (time-based example) ---
def autonomous() -> None:
	brain.screen.clear_screen()
	brain.screen.set_cursor(1, 1)
	brain.screen.print("Auton: Push Back (paths)")

	if USE_ODOMETRY:
		# Initialize and start odometry thread
		_odom.reset(0.0, 0.0, 0.0)
		start_odometry_thread()
		brain.screen.new_line()
		brain.screen.print("Odometry ON")

	# Load all four routes from path.txt
	paths = load_all_paths_from_file()
	if not paths or START_PATH_INDEX >= len(paths):
		# Fallback to simple example routine if file missing or index out of range
		brain.screen.new_line()
		brain.screen.print("Fallback: simple routine")

		operate_intake(True, 90)
		drive_time(1200, 65, forward=True)
		operate_intake(False)
		drive_time(400, 40, forward=False)
		turn_time(700, 55, left=True)
		drive_time(1000, 70, forward=True)
		operate_intake(True, 40)
		wait(300, MSEC)
		operate_intake(False)
		drive_time(700, 60, forward=False)
		brain.screen.new_line()
		brain.screen.print("Auton complete")
		return

	# Select the desired route and follow it (abort if exceeds 15s budget)
	route = paths[START_PATH_INDEX]
	brain.screen.new_line()
	brain.screen.print("Route index: {}".format(START_PATH_INDEX))
	if USE_ODOMETRY:
		follow_path_odometry(route,
						  DRIVE_SPEED_PCT,
						  TURN_SPEED_PCT,
						  ODOM_DIST_TOL_IN,
						  ODOM_HEAD_TOL_DEG,
						  max_ms=AUTON_DURATION_MS - 1000)
	else:
		follow_path_time_based(route,
					   PATH_STRIDE,
					   DRIVE_SPEED_PCT,
					   TURN_SPEED_PCT,
					   max_ms=AUTON_DURATION_MS - 1000)  # leave buffer
	brain.screen.new_line()
	brain.screen.print("Auton complete")


def _cap(v: float, lo: float = -100.0, hi: float = 100.0) -> float:
	if v < lo:
		return lo
	if v > hi:
		return hi
	return v


def driver_control() -> None:
	"""Driver Controlled Period (1:45). Arcade drive + intake/lift controls.

	Controls (adjust as desired):
	- Left stick (Axis3): forward/back
	- Right stick (Axis1): turn left/right
	- R1: intake in
	- R2: intake out
	- L1: lift up (optional)
	- L2: lift down (optional)

	Endgame helper: rumbles the controller at ~1:25 to remind of SG11 (protected Park Zones in last 0:20).
	"""
	# Prepare motors
	left_motor.set_stopping(BRAKE)
	right_motor.set_stopping(BRAKE)
	intake.set_stopping(COAST)
	lift.set_stopping(BRAKE)

	# Timer to alert for last 20 seconds
	t = Timer()
	endgame_alerted = False

	brain.screen.clear_screen()
	brain.screen.set_cursor(1, 1)
	brain.screen.print("Driver: Push Back")

	while True:
		# Arcade mixing
		fwd = controller.axis3.position()
		turn = controller.axis1.position()
		left_cmd = _cap(fwd + turn)
		right_cmd = _cap(fwd - turn)

		left_motor.set_velocity(left_cmd, PERCENT)
		right_motor.set_velocity(right_cmd, PERCENT)
		left_motor.spin(FORWARD)
		right_motor.spin(FORWARD)

		# Intake controls
		if controller.buttonR1.pressing():
			intake.spin(REVERSE, 100, PERCENT)  # Outtake
		elif controller.buttonR2.pressing():
			intake.spin(FORWARD, 100, PERCENT)  # Intake
		else:
			intake.stop(COAST)

		# Lift controls (if wired)
		if controller.buttonL1.pressing():
			lift.spin(FORWARD, 80, PERCENT)
		elif controller.buttonL2.pressing():
			lift.spin(REVERSE, 80, PERCENT)
		else:
			lift.stop(BRAKE)

		# Endgame reminder at last 20s (SG11 protected Park Zones)
		if not endgame_alerted and t.time(MSEC) >= ENDGAME_WARN_MS:
			try:
				controller.rumble("..")
			except Exception:
				pass
			endgame_alerted = True

		wait(20, MSEC)


# Competition template wiring
competition = Competition(driver_control, autonomous)


if __name__ == "__main__":
	# uncomment to run either directly:
	# autonomous()
	# driver_control()


	# Program selector for standalone testing and pre-auton selection
	# This lets teams choose which preset route to run and whether to use odometry
	# before starting autonomous (or to run driver control manually).
	def program_selector(timeout_s: float = 15.0):
		global START_PATH_INDEX

		# load information
		paths = load_all_paths_from_file()
		routes_count = len(paths)

		start_t = Timer()
		brain.screen.clear_screen()
		brain.screen.set_cursor(1, 1)
		brain.screen.print("Program Selector")
		while True:
			# Render status
			brain.screen.set_cursor(2, 1)
			if routes_count:
				brain.screen.print(f"Route {START_PATH_INDEX+1}/{routes_count}   ")
			else:
				brain.screen.print("Route (none)        ")
			brain.screen.new_line()
			# Odometry is always enabled in this simplified selector
			brain.screen.print("Odometry: ON   ")
			brain.screen.new_line()
			brain.screen.print("A=Auton  B=Driver")

			# Controller inputs
			if controller.buttonLeft.pressing():
				# decrements route index
				if routes_count:
					START_PATH_INDEX = max(0, (START_PATH_INDEX - 1) % routes_count)
					wait(200, MSEC)
			elif controller.buttonRight.pressing():
				if routes_count:
					START_PATH_INDEX = (START_PATH_INDEX + 1) % routes_count
					wait(200, MSEC)
			# Note: Odometry toggle removed - odometry is enforced ON by default
			elif controller.buttonA.pressing():
				# Start autonomous
				brain.screen.clear_screen()
				brain.screen.print("Auton Selected")
				wait(200, MSEC)
				return "auton"
			elif controller.buttonB.pressing():
				# Start driver
				brain.screen.clear_screen()
				brain.screen.print("Driver Selected")
				wait(200, MSEC)
				return "driver"

			# Timeout handling
			if timeout_s is not None and start_t.time(SEC) >= timeout_s:
				brain.screen.clear_screen()
				brain.screen.print("No selection: Driver")
				wait(200, MSEC)
				return "driver"

			wait(20, MSEC)

	# Use selector when running standalone
	selection = program_selector(timeout_s=20.0)
	if selection == "auton":
		autonomous()
	else:
		driver_control()

        
