# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       someon                                                       #
# 	Created:      11/11/2025,                                                  #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *
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


# --- Autonomous routine (time-based example) ---
def autonomous() -> None:
	brain.screen.clear_screen()
	brain.screen.set_cursor(1, 1)
	brain.screen.print("Auton: Push Back (paths)")

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
			intake.spin(FORWARD, 100, PERCENT)
		elif controller.buttonR2.pressing():
			intake.spin(REVERSE, 100, PERCENT)
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


	# Standalone chooser for quick testing without field control.
	# Press Controller A for Autonomous, B for Driver.
	# brain.screen.clear_screen()
	# brain.screen.set_cursor(1, 1)
	# brain.screen.print("Select mode:")
	# brain.screen.new_line()
	# brain.screen.print("A=Auton  B=Driver")

	# selection = None
	# start_t = Timer()
	# while selection is None and start_t.time(MSEC) < 10_000:  # wait up to 10s
	# 	if controller.buttonA.pressing():
	# 		selection = "auton"
	# 	elif controller.buttonB.pressing():
	# 		selection = "driver"
	# 	wait(20, MSEC)

	# if selection == "auton":
	# 	autonomous()
	# else:
	# 	# default to driver if none selected
	# 	driver_control()

        
