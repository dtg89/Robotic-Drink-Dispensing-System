"""
This script is designed as a small Arm Script module (arm.py -
Hiwonder xArm 1S pick / place / serve sequences for the RDDS (Raspberry Pi 4))
that is available to be imported into the main firmware of this system.

- Connection: xArm 1S controller -> Pi over USB, using the 'xarm' Python package
    pip install xarm hidapi   (on the Pi; add --break-system-packages if pip asks)
    If you get a USB permission error, add a udev rule for VID 0483 / PID 5750
    (do not use sudo: it cannot see packages installed in your user account).

- How the motion works (no inverse kinematics):
    You pose the arm once ("teach") by typing small jog commands, and the script
    saves the servo positions (0-1000 units, 500 = center) in arm_poses.json:
      Poses (servos 2-6): INITIAL_PICKUP, SCALE_DROP, SCALE_PICKUP, FINAL_DROP, RETURN
      Gripper (servo 1):  open, closed (closed is taught on the glass)
      LIFT: one small "raise the glass" offset, taught once and applied at every stop
            so the glass is lifted clear before it travels and never dragged.
    RETURN is also the HOME / rest pose. It must be clear of the scale and the nozzles.

- Export for firmware:
    from arm import XArm
    arm = XArm()
    arm.home()                            # claw open, move to RETURN
    arm.move_to_start_and_grab()          # pick the empty glass (INITIAL_PICKUP)
    arm.move_to_scale_and_release()       # place it on the scale (SCALE_DROP), back to RETURN
    arm.grab_and_move_to_end()            # SCALE_PICKUP -> FINAL_DROP, back to RETURN

- Bench tests (run on the Pi, arm clear and a hand ready near the power):
    python3 arm.py probe                  # nudges each servo in turn: find which ID is the claw
    python3 arm.py read                   # prints the current servo positions once
    python3 arm.py teach                  # guided: jog the arm into each pose, type ok to save
    python3 arm.py teach SCALE_DROP       # re-teach one pose (also: open, closed, LIFT, ...)
    python3 arm.py list                   # show the saved poses
    python3 arm.py go RETURN              # move to one pose (add 'hover' for the lifted version)
    python3 arm.py grip open|closed       # gripper only
    python3 arm.py run pick               # RETURN -> pick up glass -> place on scale -> RETURN
    python3 arm.py run serve              # scale -> serving area -> RETURN

- Jog commands inside 'teach' (units: 1000 = full travel, 10 is about 2.4 degrees):
    3+  /  3-         move servo 3 by the current step
    3 +50  /  3 -50   move servo 3 by 50
    3 =520            move servo 3 to position 520
    step 60           change the default step
    show              print the current positions
    ok                save this pose and go on
    q                 quit (poses saved so far are kept)
"""

import json
import os
import re
import sys
import time

GRIPPER_ID = 1                      # claw servo (confirmed with: python3 arm.py probe)
JOINT_IDS = [2, 3, 4, 5, 6]         # wrist rotate .. base (taught as one pose)
MOVE_MS = 3000                      # travel between stops: slow, nothing spills
LIFT_MS = 1500                      # the short up / down move at a stop
EXIT_EXTRA = -40                    # extra lift after letting go at the serving spot (more negative = higher)
CARRY_MS = 5000                     # carrying the filled glass to the serving spot: very slow
GRIP_MS = 600
POSES_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_poses.json")

POSE_NAMES = ["INITIAL_PICKUP", "SCALE_DROP", "SCALE_PICKUP", "FINAL_DROP", "RETURN"]
TEACH_HINTS = {
    "INITIAL_PICKUP": "Claw open, lowered so the empty glass sits between the fingers",
    "LIFT":           "Glass in the claw: raise it 2 cm straight up (clear of the table), keep the claw facing the same way",
    "SCALE_DROP":     "Carry the glass to the scale and lower it until it just touches the platform mark "
                      "(approach from the side: the glass must stay BELOW the nozzle tips)",
    "SCALE_PICKUP":   "The glass is already there. Just type ok (or nudge it if the claw should sit differently)",
    "FINAL_DROP":     "Lift the glass, carry it to the serving spot and lower it onto the spot",
    "RETURN":         "The claw opens and lets go of the glass. Raise and fold the arm clear of the scale and the nozzles",
}


def _clamp(pos):
    return max(0, min(1000, int(pos)))


class XArm:
    def __init__(self, controller=None, poses_file=POSES_FILE):
        if controller is None:
            import xarm
            controller = xarm.Controller("USB")
        self.arm = controller
        self.at_scale = False       # True while the arm waits at the scale with the glass
        try:
            with open(poses_file) as f:
                data = json.load(f)
            self.poses, self.grip = data["poses"], data["grip"]
            self.lift = data.get("lift", {})
            missing = [n for n in POSE_NAMES if n not in self.poses] + \
                      [g for g in ("open", "closed") if g not in self.grip]
            if missing or not self.lift:
                raise KeyError(missing or "LIFT")
        except (FileNotFoundError, KeyError, ValueError):
            raise RuntimeError("Arm poses not taught yet (or incomplete). Run: python3 arm.py teach")
        print(f"[INIT] xArm connected, {len(self.poses)} poses loaded")

    # Low-level moves
    def _move(self, name, ms=MOVE_MS, hover=False, extra=0):
        """Move to a taught pose. hover=True: the same pose lifted by the taught LIFT offset."""
        pairs = []
        for sid, pos in self.poses[name].items():
            if hover:
                pos = pos + self.lift.get(sid, 0) + (extra if sid == "5" else 0)
            pairs.append([int(sid), _clamp(pos)])
        print(f"[ARM] -> {name}{' (lifted)' if hover else ''}")
        self.arm.setPosition(pairs, duration=ms, wait=True)

    def _grip(self, state):
        pos = _clamp(self.grip[state])
        print(f"[ARM] gripper {state}")
        self.arm.setPosition([[GRIPPER_ID, pos]], duration=GRIP_MS, wait=True)

    # Sequences (names match the firmware)
    def home(self):
        self._grip("open")
        if self.at_scale:                                  # waiting at the scale (e.g. after a fault): rise first
            self._move("SCALE_DROP", ms=LIFT_MS, hover=True, extra=EXIT_EXTRA)
        self._move("RETURN")
        self.at_scale = False

    def move_to_start_and_grab(self):
        self._grip("open")
        self._move("INITIAL_PICKUP", hover=True)
        self._move("INITIAL_PICKUP", ms=LIFT_MS)
        self._grip("closed")
        self._move("INITIAL_PICKUP", ms=LIFT_MS, hover=True)

    def move_to_scale_and_release(self):
        """Set the glass on the platform and let go. The arm then WAITS here, claw open around
        the glass and clear of the nozzles, while the pumps pour."""
        self._move("SCALE_DROP", hover=True)
        self._move("SCALE_DROP", ms=LIFT_MS)
        self._grip("open")
        self.at_scale = True

    def grab_and_move_to_end(self):
        if self.at_scale:                                  # already waiting around the glass
            self._move("SCALE_PICKUP", ms=LIFT_MS)
        else:                                              # coming from elsewhere: approach lifted
            self._move("SCALE_PICKUP", hover=True)
            self._move("SCALE_PICKUP", ms=LIFT_MS)
        self._grip("closed")
        self._move("SCALE_PICKUP", ms=LIFT_MS, hover=True)
        self._move("FINAL_DROP", ms=CARRY_MS, hover=True)
        self._move("FINAL_DROP", ms=LIFT_MS)
        self._grip("open")
        self._move("FINAL_DROP", ms=LIFT_MS, hover=True, extra=EXIT_EXTRA)   # rise clear of the glass
        self._move("RETURN")
        self.at_scale = False

# Bench Tests
def _read_pos(c, sid, tries=20):
    """
    One servo position (clamped to 0-1000). The library's getPosition() takes whatever
    reply is waiting, which can be missing or belong to another servo. So: clear stale
    replies, ask, and accept only a reply that carries this servo's ID. Retry otherwise.
    """
    for _ in range(tries):
        try:
            while c._device.read(64):                     # drop stale replies
                pass
            c._send(c.CMD_GET_SERVO_POSITION, bytearray([1, sid]))
            data = c._recv(c.CMD_GET_SERVO_POSITION)
            if data is not None and len(data) >= 4 and data[1] == sid:
                raw = data[3] * 256 + data[2]
                if raw > 32767:                           # negative, read as unsigned
                    raw -= 65536
                return _clamp(raw)
        except Exception:                                 # IndexError on an empty reply etc.
            pass
        time.sleep(0.03)
    raise RuntimeError(f"No valid reply from servo {sid}. Check the arm power and the USB cable.")


def _positions(c):
    return {sid: _read_pos(c, sid) for sid in range(1, 7)}


def _hold(c, pos):
    """Re-power every servo at its current position (a servo that went limp holds again)."""
    c.setPosition([[sid, _clamp(p)] for sid, p in pos.items()], duration=500, wait=True)


def _show(pos):
    print("  " + "  ".join(f"S{sid}:{p:4d}" for sid, p in sorted(pos.items())))


def _move_servo(c, pos, sid, new):
    new = _clamp(new)
    ms = min(1500, max(300, int(abs(new - pos[sid]) * 6)))
    c.setPosition([[sid, new]], duration=ms, wait=True)
    pos[sid] = new


def _jog(c, pos, title):
    """Interactive jog. Updates pos in place. Returns True on 'ok', False on 'q'."""
    print(f"\n{title}")
    print("  Commands: 3+  3-  3 +50  3 -50  3 =520  step 60  show  ok  q")
    step = 30
    _show(pos)
    while True:
        line = input("  jog> ").strip().lower()
        if line == "ok":
            return True
        if line == "q":
            return False
        if line == "show":
            _show(pos)
            continue
        m = re.fullmatch(r"step\s*(\d+)", line)
        if m:
            step = max(1, int(m.group(1)))
            print(f"  step = {step}")
            continue
        m = re.fullmatch(r"([1-6])\s*=\s*(\d+)", line)
        if m:
            _move_servo(c, pos, int(m.group(1)), int(m.group(2)))
            print(f"  S{m.group(1)} -> {pos[int(m.group(1))]}")
            continue
        m = re.fullmatch(r"([1-6])\s*([+-])\s*(\d*)", line)
        if m:
            sid = int(m.group(1))
            amount = int(m.group(3)) if m.group(3) else step
            _move_servo(c, pos, sid, pos[sid] + (amount if m.group(2) == "+" else -amount))
            print(f"  S{sid} -> {pos[sid]}")
            continue
        print("  ? try 3+  3-  3 +50  3 =520  step 60  show  ok  q")


def _tool_read(c):
    _show(_positions(c))


def _tool_probe(c):
    pos = _positions(c)
    _hold(c, pos)
    print("Each servo moves a little and comes back. Keep hands clear and watch which part moves.")
    for sid in range(1, 7):
        input(f"\nPress Enter to nudge servo {sid}... ")
        p = pos[sid]
        q = p + 30 if p < 500 else p - 30
        c.setPosition([[sid, _clamp(q)]], duration=400, wait=True)
        time.sleep(0.3)
        c.setPosition([[sid, p]], duration=400, wait=True)
        print(f"  Servo {sid} moved and returned. Which part of the arm was it?")
    print("\nThe claw's servo number goes in GRIPPER_ID at the top of arm.py (default 1).")


def _save_poses(poses, grip, lift):
    with open(POSES_FILE, "w") as f:
        json.dump({"poses": poses, "grip": grip, "lift": lift}, f, indent=2)


def _tool_teach(c, only=None):
    """
    Teach in the order the glass travels. The glass stays in the claw from the moment the claw
    closes (after INITIAL_PICKUP) until RETURN, where the claw opens and lets go.
    'python3 arm.py teach SCALE_DROP' re-teaches just one item ('open', 'closed', 'LIFT', or a pose).
    """
    try:
        with open(POSES_FILE) as f:
            data = json.load(f)
        poses, grip, lift = data["poses"], data["grip"], data.get("lift", {})
    except (FileNotFoundError, KeyError, ValueError):
        poses, grip, lift = {}, {}, {}
    steps = [("grip", "open"), ("pose", "INITIAL_PICKUP"), ("grip", "closed"), ("lift", "LIFT"),
             ("pose", "SCALE_DROP"), ("pose", "SCALE_PICKUP"), ("pose", "FINAL_DROP"), ("pose", "RETURN")]
    if only:
        steps = [s for s in steps if s[1].upper() == only.upper()]
        if not steps:
            print(f"Unknown item '{only}'. Use one of: open, closed, LIFT, " + ", ".join(POSE_NAMES))
            return
    else:
        print("Put the EMPTY glass at the pick-up spot first. The claw closes on it after INITIAL_PICKUP,")
        print("and it stays in the claw until RETURN, where the claw opens and lets go.")
    print("TEACH MODE: the arm holds its position. Jog it with small commands (listed below).")
    pos = _positions(c)
    _hold(c, pos)
    for kind, name in steps:
        if kind == "grip" and name == "open":
            title = "GRIPPER OPEN (servo %d): open the claw wide enough to go around the glass" % GRIPPER_ID
        elif kind == "grip":
            title = "GRIPPER CLOSED (servo %d): the glass is between the fingers: close until it grips (not crushing)" % GRIPPER_ID
        else:
            title = f"{name}: {TEACH_HINTS[name]}"
        if kind == "lift":
            base = poses.get("INITIAL_PICKUP")
            if base is None:
                print("Teach INITIAL_PICKUP first.")
                return
            if only:                                   # re-teaching just LIFT: start at the pick-up pose
                for sid, p in base.items():
                    pos[int(sid)] = _clamp(p)
                _hold(c, pos)
        if name == "RETURN" and "open" in grip:
            print("  (opening the claw to let go of the glass)")
            _move_servo(c, pos, GRIPPER_ID, grip["open"])
        if not _jog(c, pos, title):
            print("Stopped. Poses saved so far are kept.")
            return
        if kind == "grip":
            grip[name] = pos[GRIPPER_ID]
            print(f"  saved gripper {name} = {grip[name]}")
        elif kind == "lift":
            lift = {str(s): pos[s] - base[str(s)] for s in JOINT_IDS}
            print(f"  saved LIFT offset = {lift}")
        else:
            poses[name] = {str(s): pos[s] for s in JOINT_IDS}
            print(f"  saved {name} = {poses[name]}")
        _save_poses(poses, grip, lift)                # progress is kept after every step
    print(f"\nSaved to {POSES_FILE}. Test with:  python3 arm.py go RETURN   then   python3 arm.py run pick")


def main():
    usage = __doc__
    if len(sys.argv) < 2:
        print(usage)
        return 1
    cmd = sys.argv[1]
    if cmd in ("read", "probe", "teach"):
        import xarm
        c = xarm.Controller("USB")
        if cmd == "teach":
            _tool_teach(c, sys.argv[2] if len(sys.argv) > 2 else None)
        else:
            {"read": _tool_read, "probe": _tool_probe}[cmd](c)
        return 0
    arm = XArm()
    try:
        if cmd == "list":
            print(json.dumps({"poses": arm.poses, "grip": arm.grip, "lift": arm.lift}, indent=2))
        elif cmd == "go":
            arm._move(sys.argv[2].upper(), hover=(len(sys.argv) > 3 and sys.argv[3] == "hover"))
        elif cmd == "grip":
            arm._grip(sys.argv[2].lower())
        elif cmd == "run" and sys.argv[2] == "pick":
            arm.home(); arm.move_to_start_and_grab(); arm.move_to_scale_and_release()
        elif cmd == "run" and sys.argv[2] == "serve":
            arm.grab_and_move_to_end()
        elif cmd == "run" and sys.argv[2] == "all":      # the whole cycle, pause = the pumps pouring
            pause = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
            arm.home(); arm.move_to_start_and_grab(); arm.move_to_scale_and_release()
            print(f"[ARM] (pause {pause:.0f} s: the pumps would pour here)")
            time.sleep(pause)
            arm.grab_and_move_to_end()
        else:
            print(usage)
            return 1
    except KeyboardInterrupt:
        print("\nInterrupted: the arm stays where it is.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
