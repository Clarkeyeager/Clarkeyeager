# theskyx_dome_bridge.py
# Date: 4/4/2026

import socket
import serial
import time
import math
import msvcrt

TSX_HOST = "localhost"
TSX_PORT = 3040
DOME_PORT = "COM5"        # Change to your dome's COM port
POLL_INTERVAL = 5         # Check telescope every 5 seconds
ROTATION_THRESHOLD = 3    # Send update when dome azimuth changes by this many degrees
SLEW_THRESHOLD = 10       # Degrees change that indicates a new slew (fast response)
HORIZON_ALTITUDE = 10     # Altitude (degrees) that triggers park/close sequence
PARK_CLOSE_DELAY = 60     # Seconds to wait for dome to reach park before closing shutter
AUTO_CLOSE_SHUTTER = True # Set to False to park only, without closing shutter
STARTUP_HOME_DELAY = 60   # Seconds to wait for dome to home/park on startup before opening shutter
AUTO_OPEN_SHUTTER = True  # Set to False to skip opening shutter on startup
FLIP_POLL_INTERVAL = 10   # Seconds between polls during a flip (longer so a brief mid-flip pause won't look like settled)
FAST_MOTION_THRESHOLD = 0.5  # deg/5sec — scope moving faster than this (slew or flip) blocks dome commands and altitude check

# Dome geometry - measure these values for your observatory
# All distances in meters, angles in degrees
DOME_RADIUS    = 1.17     # Dome interior radius
MOUNT_DX       = 0.13    # Mount RA axis offset from dome center: + = East,  - = West
MOUNT_DY       = 0.064   # Mount RA axis offset from dome center: + = North, - = South
MOUNT_DZ       = 0.33    # Mount RA axis height above dome center height: + = up
LATITUDE       = 40.1167  # Observer latitude in degrees North (40 deg 07 min)
DEC_ARM_LENGTH = 0.24    # Distance from RA axis to OTA mounting point (meters)
                       # Set to 0 to disable GEM arm correction


def compute_ha(az_deg, alt_deg, dec_deg):
    """Compute hour angle (radians) from azimuth, altitude, and declination.

    Uses spherical trigonometry. Result is positive westward (standard convention).
    Returns 0.0 near the celestial pole where HA is undefined.
    """
    az  = math.radians(az_deg)
    alt = math.radians(alt_deg)
    dec = math.radians(dec_deg)
    lat = math.radians(LATITUDE)

    cos_dec = math.cos(dec)
    if abs(cos_dec) < 1e-6:
        return 0.0  # Near celestial pole, HA undefined

    sin_ha = -math.sin(az) * math.cos(alt) / cos_dec
    cos_ha = (math.sin(alt) - math.sin(dec) * math.sin(lat)) / (cos_dec * math.cos(lat))
    return math.atan2(sin_ha, cos_ha)


def calc_dome_azimuth(scope_az_deg, scope_alt_deg, scope_dec_deg, pier_side):
    """Calculate dome slit azimuth with full GEM arm geometry.

    Accounts for:
    - Mount RA axis offset from dome center (MOUNT_DX/DY/DZ)
    - GEM declination arm swinging around polar axis with hour angle (DEC_ARM_LENGTH)
    - Pier side (east/west) flip of the OTA arm direction

    With DEC_ARM_LENGTH = 0 the result is identical to the simple offset formula.
    With all offsets = 0 the result equals the scope azimuth.
    """
    az  = math.radians(scope_az_deg)
    alt = math.radians(scope_alt_deg)
    lat = math.radians(LATITUDE)

    # Compute hour angle from current pointing
    ha = compute_ha(scope_az_deg, scope_alt_deg, scope_dec_deg)

    # GEM dec arm direction — unit vector from RA axis toward OTA mounting point.
    # Derived by rotating the reference arm direction (-1,0,0) around the polar
    # axis (0, cos(lat), sin(lat)) by the hour angle using Rodrigues formula.
    # pier_side=1 (pier east): OTA on west side, arm points west at HA=0
    # pier_side=2 (pier west): OTA on east side, arm points east at HA=0
    arm_x = -math.cos(ha)
    arm_y = -math.sin(lat) * math.sin(ha)
    arm_z =  math.cos(lat) * math.sin(ha)
    if pier_side != 1:   # Pier west: flip arm to opposite side
        arm_x, arm_y, arm_z = -arm_x, -arm_y, -arm_z

    # Position of optical axis origin (OTA mounting point on dec axis)
    ox = MOUNT_DX + DEC_ARM_LENGTH * arm_x
    oy = MOUNT_DY + DEC_ARM_LENGTH * arm_y
    oz = MOUNT_DZ + DEC_ARM_LENGTH * arm_z

    # Telescope pointing unit vector (az=0 is North, az=90 is East)
    Tx = math.sin(az) * math.cos(alt)   # East component
    Ty = math.cos(az) * math.cos(alt)   # North component
    Tz = math.sin(alt)                  # Up component

    # Find intersection of optical axis ray with dome sphere.
    # Ray from (ox,oy,oz) in direction (Tx,Ty,Tz), sphere radius DOME_RADIUS.
    # Quadratic: t^2 + 2*b*t + c = 0
    b = ox*Tx + oy*Ty + oz*Tz
    c = ox**2 + oy**2 + oz**2 - DOME_RADIUS**2

    discriminant = b**2 - c
    if discriminant < 0:
        print(f"WARNING: dome geometry error (discriminant < 0), using scope azimuth")
        return scope_az_deg

    t = -b + math.sqrt(discriminant)  # Positive root = forward intersection

    # Dome slit intersection point
    Px = ox + t * Tx
    Py = oy + t * Ty

    dome_az = math.degrees(math.atan2(Px, Py)) % 360
    return dome_az


def tsx_send(command):
    # TheSkyX 10.5 requires the script wrapped in packet markers
    wrapped = "/* Java Script */ /* Socket Start Packet */ " + command + " /* Socket End Packet */"
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(10)
    s.connect((TSX_HOST, TSX_PORT))
    s.sendall((wrapped + "\n").encode())
    response = s.recv(4096).decode()
    s.close()
    return response

def get_telescope_status():
    """Get azimuth, altitude, declination, and pier side in a single query."""
    js = "var isConn = sky6RASCOMTele.IsConnected; sky6RASCOMTele.GetAzAlt(); sky6RASCOMTele.GetRaDec(); var Out = isConn + '|' + sky6RASCOMTele.dAz + '|' + sky6RASCOMTele.dAlt + '|' + sky6RASCOMTele.dDec; Out"
    result = tsx_send(js)
    raw = result  # Keep original for diagnostics
    # Strip trailing TheSkyX status message (e.g. "no error. error = 0.")
    if "no error" in result.lower():
        result = result[:result.lower().index("no error")].rstrip("|").rstrip()
    parts = result.split("|")
    if len(parts) < 4:
        if "another script is running" in raw.lower():
            raise ValueError("TheSkyX busy: another script is running - use Script > Stop in TheSkyX to clear it")
        raise ValueError(f"Bad TheSkyX response (raw='{raw.strip()}')")
    if parts[0].strip() == '0':
        raise ValueError("Telescope not connected in TheSkyX (IsConnected=0)")
    az  = float(parts[1])
    alt = float(parts[2])
    dec = float(parts[3])
    # jnPierSide not supported in TheSkyX 10.5 - derive from hour angle
    # HA >= 0 (object west of meridian) = pier east (1)
    # HA <  0 (object east of meridian) = pier west (2)
    ha = compute_ha(az, alt, dec)
    pier_side = 1 if ha >= 0 else 2
    return az, alt, pier_side, dec

dome = serial.Serial(DOME_PORT, 9600)
time.sleep(2)  # Wait for Arduino to initialize after serial connection

# One-time startup sequence: home dome for calibration, then open shutter if needed
ans = input("Home dome for calibration? (Y/n): ").strip().lower()
if ans != 'n':
    print("Startup: Homing dome for calibration...")
    dome.reset_input_buffer()
    dome.write(b"G")
    print(f"Waiting for dome to reach park position (timeout {STARTUP_HOME_DELAY} sec)...")
    home_start = time.time()
    home_buf = ""
    while time.time() - home_start < STARTUP_HOME_DELAY:
        if dome.in_waiting:
            home_buf += dome.read(dome.in_waiting).decode(errors='ignore')
            if "Park complete." in home_buf:
                print(f"Dome parked ({int(time.time() - home_start)} sec)")
                break
        time.sleep(0.5)
    else:
        print(f"ERROR: Homing did not complete within {STARTUP_HOME_DELAY} sec - dome position unknown. Bridge stopped.")
        raise SystemExit(1)
else:
    print("Startup: Skipping home/calibration")

if AUTO_OPEN_SHUTTER:
    # Flush any stale homing output before querying shutter state
    dome.reset_input_buffer()
    dome.write(b"S")
    time.sleep(2)  # Wait for full status response at 9600 baud
    status_response = ""
    while dome.in_waiting:
        status_response += dome.read(dome.in_waiting).decode(errors='ignore')
        time.sleep(0.1)
    if "Shutter state: OPEN" in status_response:
        print("Startup: Shutter already open - skipping open sequence")
    else:
        print("Startup: Opening shutter...")
        dome.write(b"O")
print("Startup sequence complete - beginning tracking")

last_sent_az = -1         # Last telescope azimuth sent (for slew detection)
last_sent_dome_az = -1    # Last dome azimuth sent (for rotation tracking)
last_polled_az = -1       # Last polled azimuth integer (for flip detection)
last_polled_az_f = -1.0   # Last polled azimuth float (for flip settle detection)
flip_in_progress = False  # True while mount is physically flipping
was_slewing = False       # True after scope_moving goes high — signals a real user slew occurred
physical_pier_side = -1   # Actual physical pier side — only changes on confirmed physical flip
horizon_shutdown_sent = False

FLIP_MOTION_THRESHOLD = 1.0   # deg/poll — motion above this while past meridian triggers flip hold

# Flush any keystrokes typed during startup before entering main loop
while msvcrt.kbhit():
    msvcrt.getwch()

print("Dome bridge started - monitoring TheSkyX telescope position...")
print("Press Q at any time to initiate end-of-night shutdown.")

while True:
    try:
        azimuth, altitude, pier_side, declination = get_telescope_status()
        az_int = int(azimuth + 0.5) % 360          # Telescope azimuth (for slew detection)

        # Compute scope motion rate — blocks dome commands and altitude check when moving faster than tracking
        if last_polled_az_f >= 0:
            az_rate = abs(azimuth - last_polled_az_f)
            if az_rate > 180:
                az_rate = 360 - az_rate
        else:
            az_rate = 0.0
        scope_moving = az_rate > FAST_MOTION_THRESHOLD
        if scope_moving:
            was_slewing = True  # Remember that a real slew occurred

        # Set physical pier side from HA on first poll only
        # After that it only changes when a physical flip is confirmed
        if physical_pier_side == -1:
            physical_pier_side = pier_side
            print(f"Initial pier side set to {'east' if physical_pier_side == 1 else 'west'}")

        # Use physical pier side for dome geometry — not HA-derived pier side
        # This prevents dome from moving when scope drifts past meridian without flipping
        dome_az = calc_dome_azimuth(azimuth, altitude, declination, physical_pier_side)
        dome_az_int = int(dome_az + 0.5) % 360     # Corrected dome azimuth (sent to controller)

        # Detect flip: rapid motion (> FLIP_MOTION_THRESHOLD deg/poll) AND HA-derived pier side
        # disagrees with confirmed physical pier side.
        # After a completed flip both match, so no false re-detection.
        # Ignore additional motion detections while flip is already in progress.
        if last_polled_az >= 0 and not flip_in_progress:
            az_change = abs(az_int - last_polled_az)
            if az_change > 180:
                az_change = 360 - az_change
            if az_change > FLIP_MOTION_THRESHOLD and pier_side != physical_pier_side:
                flip_in_progress = True
                print(f"Flip detected - scope moving ({az_change:.2f} deg/poll), pier side changed - holding dome...")

        # Reset shutdown flag when object rises back above threshold (2 deg hysteresis)
        if horizon_shutdown_sent and altitude > HORIZON_ALTITUDE + 2:
            horizon_shutdown_sent = False
            print(f"Target back above horizon ({altitude:.1f} deg) - resuming tracking")

        if altitude <= HORIZON_ALTITUDE and not scope_moving:
            if not horizon_shutdown_sent:
                print(f"Target below {HORIZON_ALTITUDE} deg ({altitude:.1f} deg) - parking dome")
                dome.reset_input_buffer()
                dome.write(b"G")
                park_buf = ""
                park_start = time.time()
                parked = False
                while time.time() - park_start < PARK_CLOSE_DELAY:
                    if dome.in_waiting:
                        park_buf += dome.read(dome.in_waiting).decode(errors='ignore')
                        if "Park complete." in park_buf:
                            print(f"Dome parked ({int(time.time() - park_start)} sec)")
                            parked = True
                            break
                    time.sleep(0.5)
                if not parked:
                    print(f"WARNING: Dome did not reach park within {PARK_CLOSE_DELAY} sec - position unknown")
                if AUTO_CLOSE_SHUTTER:
                    dome.reset_input_buffer()
                    time.sleep(0.5)
                    print("Closing shutter...")
                    dome.write(b"C")
                    close_buf = ""
                    close_start = time.time()
                    while time.time() - close_start < 150:
                        if dome.in_waiting:
                            close_buf += dome.read(dome.in_waiting).decode(errors='ignore')
                            if "Shutter CLOSE sequence complete." in close_buf:
                                print("Shutter closed.")
                                break
                            if "already CLOSED" in close_buf:
                                print("Shutter already closed.")
                                break
                        time.sleep(0.5)
                    else:
                        print("WARNING: Shutter close not confirmed - check shutter manually")
                horizon_shutdown_sent = True
            time.sleep(POLL_INTERVAL)
            continue

        # While flip is in progress, skip all dome logic — just wait for mount to settle.
        # We already have azimuth/pier_side from get_telescope_status() above; no extra query needed.
        if flip_in_progress:
            if last_polled_az_f >= 0:
                az_change_f = abs(azimuth - last_polled_az_f)
                if az_change_f > 180:
                    az_change_f = 360 - az_change_f
                if az_change_f < 0.5:
                    # Mount has settled - update physical pier side to match new position
                    physical_pier_side = pier_side
                    flip_in_progress = False
                    # Recalculate dome with confirmed new physical pier side
                    dome_az = calc_dome_azimuth(azimuth, altitude, declination, physical_pier_side)
                    dome_az_int = int(dome_az + 0.5) % 360
                    print(f"Flip complete (pier {'east' if physical_pier_side == 1 else 'west'}) - moving dome to {dome_az_int} deg")
                    dome.write((str(dome_az_int) + "\n").encode())
                    last_sent_az = az_int
                    last_sent_dome_az = dome_az_int
                else:
                    print(f"Flip in progress - scope moved {az_change_f:.3f} deg/{FLIP_POLL_INTERVAL}s, holding dome...")
            last_polled_az = az_int
            last_polled_az_f = azimuth
            time.sleep(FLIP_POLL_INTERVAL)
            continue

        last_polled_az = az_int
        last_polled_az_f = azimuth

        # Detect new slew (large telescope position change)
        if last_sent_az >= 0:
            slew_diff = abs(az_int - last_sent_az)
            if slew_diff > 180:
                slew_diff = 360 - slew_diff
        else:
            slew_diff = 999  # First reading, always send

        # Detect dome rotation (dome azimuth drifted by ROTATION_THRESHOLD degrees)
        if last_sent_dome_az >= 0:
            rot_diff = abs(dome_az_int - last_sent_dome_az)
            if rot_diff > 180:
                rot_diff = 360 - rot_diff
        else:
            rot_diff = 0  # First reading handled by slew_diff above

        # Send to dome if: not moving fast, AND (new slew detected OR dome has rotated 3+ degrees)
        if not scope_moving and (slew_diff >= SLEW_THRESHOLD or rot_diff >= ROTATION_THRESHOLD):
            if slew_diff >= SLEW_THRESHOLD:
                # Scope settled at new target — update physical pier side only if the scope was
                # actually slewing (scope_moving was True). This prevents tracking drift past the
                # meridian from falsely updating pier side via accumulated slew_diff.
                if was_slewing:
                    physical_pier_side = pier_side
                    was_slewing = False
                dome_az = calc_dome_azimuth(azimuth, altitude, declination, physical_pier_side)
                dome_az_int = int(dome_az + 0.5) % 360
                print(f"New target (pier {'east' if physical_pier_side == 1 else 'west'}): scope={az_int} deg dome={dome_az_int} deg (was {last_sent_az} deg)")
            else:
                print(f"Rotation update: scope={az_int} deg dome={dome_az_int} deg (moved {rot_diff} deg)")

            dome.write((str(dome_az_int) + "\n").encode())
            last_sent_az = az_int
            last_sent_dome_az = dome_az_int

        last_polled_az = az_int
        last_polled_az_f = azimuth

    except Exception as e:
        print(f"Error: {e}")

    # Check for Q keypress to trigger end-of-night shutdown
    if msvcrt.kbhit():
        key = msvcrt.getwch()
        if key.lower() == 'q':
            print("\nShutdown for the night? (Y/n): ", end='', flush=True)
            ans = input().strip().lower()
            if ans != 'n':
                print("Shutting down - parking dome...")
                dome.reset_input_buffer()
                dome.write(b"G")
                # Wait for Arduino to confirm park or its own homing timeout.
                # Cannot send commands while Arduino is in its blocking homing routine.
                # Must keep port open — closing it resets the Arduino via USB/DTR.
                shutdown_buf = ""
                park_start = time.time()
                parked = False
                while time.time() - park_start < PARK_CLOSE_DELAY * 2:
                    if dome.in_waiting:
                        shutdown_buf += dome.read(dome.in_waiting).decode(errors='ignore')
                        if "Park complete." in shutdown_buf:
                            print(f"Dome parked ({int(time.time() - park_start)} sec)")
                            parked = True
                            break
                        if "ERROR: Homing timeout" in shutdown_buf:
                            print("WARNING: Arduino homing error - rotator stopped, dome position unknown")
                            break
                    time.sleep(0.5)
                if not parked and "ERROR: Homing timeout" not in shutdown_buf:
                    print(f"WARNING: No response from Arduino after {PARK_CLOSE_DELAY * 2} sec")
                if AUTO_CLOSE_SHUTTER:
                    dome.reset_input_buffer()  # Clear homing output before sending C
                    time.sleep(0.5)            # Let Arduino settle after homing
                    print("Closing shutter...")
                    dome.write(b"C")
                    # Wait for shutter close confirmation before exiting —
                    # exiting now would close the serial port and reset the Arduino mid-close
                    close_buf = ""
                    close_start = time.time()
                    while time.time() - close_start < 150:
                        if dome.in_waiting:
                            close_buf += dome.read(dome.in_waiting).decode(errors='ignore')
                            if "Shutter CLOSE sequence complete." in close_buf:
                                print("Shutter closed.")
                                break
                            if "already CLOSED" in close_buf:
                                print("Shutter already closed.")
                                break
                        time.sleep(0.5)
                    else:
                        print("WARNING: Shutter close not confirmed - check shutter manually")
                print("Shutdown complete.")
                raise SystemExit(0)
            else:
                print("Shutdown cancelled - continuing tracking")

    time.sleep(POLL_INTERVAL)
