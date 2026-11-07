"""
This script is designed as the main Firmware module (firmware.py -
supervisory finite state machine for the RDDS (Raspberry Pi 4)) that ties the
keypad, pump and scale modules together with the xArm sequence.

- Requires (same folder): keypad.py, pumps.py, scale.py, arm.py (real runs; run
  'python3 arm.py teach' once first)

- State flow (matches the Level 2 figure):
    INITIALIZE -> IDLE (S0, scan keypad) -> LOAD_RECIPE (S1) -> PLACE_GLASS (S2)
    -> TARE_SCALE (S3) -> DISPENSE x3 (S4-S6) -> VERIFY_TOTAL (S7)
    -> SERVE_GLASS (S8) -> COMPLETE (S9) -> IDLE
    Any error -> FAULT (all pumps LOW) -> operator reset (press a keypad key) -> INITIALIZE

- Closed-loop mass cutoff (per ingredient):
    W_net(t) = W_measured(t) - W_start
    When W_net >= W_target - W_lead: pump LOW, then hold DWELL_S (1.5 s) for the
    fluid to settle before the next state.

- Run on the Pi:
    python3 firmware.py                  # real hardware
    python3 firmware.py --sim --key 1    # no hardware: simulated scale/pumps, runs recipe 1

- Before the first real run: pumps primed, scale calibrated (python3 scale.py calibrate),
  platform EMPTY at startup (the empty platform is tared in INITIALIZE).
"""

import argparse
import random
import sys
import time
from enum import Enum, auto

from keypad import Keypad
from pumps import PumpController
from scale import Scale

# Configuration
# Recipes: (pump, target grams). Totals: 1 = 30 g, 2 = 30 g, 3 = 30 g (ratios 1:2:1, 1:1:1, 2:1:2).
RECIPES = {
    "1": [("PUMP_A", 7.5), ("PUMP_B", 15), ("PUMP_C", 7.5)],
    "2": [("PUMP_A", 10), ("PUMP_B", 10), ("PUMP_C", 10)],
    "3": [("PUMP_A", 12), ("PUMP_B", 6), ("PUMP_C", 12)],
}

W_LEAD_G = {"PUMP_A": 0.5, "PUMP_B": 0.5, "PUMP_C": 0.5}  # grams that still flow after cutoff: TUNE per pump
DWELL_S = 1.5             # settle time after each pump cutoff
POUR_TIMEOUT_S = 150.0    # max time for one ingredient (set from your measured flow rate)
NO_FLOW_S = 10.0          # if less than NO_FLOW_MIN_G arrives by then: fault (unprimed / empty / kinked)
NO_FLOW_MIN_G = 1.0
OVERFILL_PCT = 15.0       # one ingredient this far over target -> fault
VERIFY_TOL_PCT = 5.0      # total outside +/- this: warning (ER-02)
VERIFY_FAULT_PCT = 15.0   # total outside +/- this: fault
CYCLE_TARGET_S = 60.0     # ER-05 (reported, not enforced)
PLACE_SETTLE_S = 1.0      # let the scale settle after the arm places the glass


class State(Enum):
    INITIALIZE = auto()
    IDLE = auto()
    LOAD_RECIPE = auto()
    PLACE_GLASS = auto()
    TARE_SCALE = auto()
    DISPENSE = auto()
    VERIFY_TOTAL = auto()
    SERVE_GLASS = auto()
    COMPLETE = auto()
    FAULT = auto()


class FaultError(Exception):
    pass


# Arm
class Arm:
    """Print-only stand-in for --sim runs. The real arm is arm.py (XArm), same method names."""
    def home(self):
        print("[ARM] Home")

    def move_to_start_and_grab(self):
        print("[ARM] Pick up empty glass")

    def move_to_scale_and_release(self):
        print("[ARM] Place glass on scale, retract clear of the nozzles")

    def grab_and_move_to_end(self):
        print("[ARM] Grab filled glass, deliver to serving area")


# Simulation (no hardware)
class _FakeDev:
    def __init__(self, pin): self.value = 0
    def on(self): self.value = 1
    def off(self): self.value = 0
    def close(self): pass


class _SimScale:
    """Weight grows while any pump is on. Same interface as scale.Scale."""
    def __init__(self, pumps, rate_g_s=5.0):
        self.pumps, self.rate, self.poured = pumps, rate_g_s, 0.0
        self.offset, self._t = 0.0, time.monotonic()

    def _advance(self):
        now = time.monotonic()
        on = sum(d.value for d in self.pumps.devs.values())
        self.poured += self.rate * on * (now - self._t)
        self._t = now

    def get_grams(self, n=3):
        time.sleep(0.02)                                      # mimic the HX711 sample time
        self._advance()
        return self.poured + random.gauss(0, 0.05) - self.offset

    def has_cup(self, min_grams=40.0): return True
    def tare(self, n=10): self._advance(); self.offset = self.poured
    def close(self): pass


class _SimKeypad:
    def __init__(self, keys): self.keys = list(keys)
    def poll(self): return self.keys.pop(0) if self.keys else None
    def close(self): pass


# Main Control & Sequencing Engine
class DrinkDispenser:
    def __init__(self, keypad, pumps, scale, arm, once=False):
        self.keypad, self.pumps, self.scale, self.arm = keypad, pumps, scale, arm
        self.once = once              # exit after one cycle (simulation)
        self.recipe_key = None
        self.recipe = []
        self.channel = 0
        self.results = []             # (pump, target, actual)
        self.cycle_start = 0.0
        self.fault_msg = ""

    # Dispatch loop
    def run(self):
        state = State.INITIALIZE
        try:
            while state is not None:
                print(f"\n=== {state.name} ===")
                try:
                    state = getattr(self, "_s_" + state.name.lower())()
                except FaultError as e:
                    self.fault_msg, state = str(e), State.FAULT
                except Exception as e:                        # unexpected = fault too
                    self.fault_msg, state = f"{type(e).__name__}: {e}", State.FAULT
        except KeyboardInterrupt:
            print("\n[STOP] Interrupted by operator")
        finally:
            self.pumps.stop_all()

    # States
    def _s_initialize(self):
        self.pumps.stop_all()
        self.arm.home()
        self.scale.tare()                                     # tare the EMPTY platform
        self.scale.get_grams(1)                               # confirms the scale is calibrated and responding
        return State.IDLE

    def _s_idle(self):
        print("Ready. Press 1, 2 or 3 to choose a recipe...")
        while True:
            key = self.keypad.poll()                          # non-blocking
            if key in RECIPES:
                self.recipe_key = key
                return State.LOAD_RECIPE
            time.sleep(0.005)

    def _s_load_recipe(self):
        self.recipe = RECIPES[self.recipe_key]
        total = sum(t for _, t in self.recipe)
        print(f"Recipe {self.recipe_key}: {self.recipe}  (total {total} g)")
        self.channel, self.results = 0, []
        self.cycle_start = time.monotonic()
        return State.PLACE_GLASS

    def _s_place_glass(self):
        self.arm.move_to_start_and_grab()
        self.arm.move_to_scale_and_release()
        return State.TARE_SCALE

    def _s_tare_scale(self):
        time.sleep(PLACE_SETTLE_S)
        if not self.scale.has_cup():                          # > 40 g over the empty platform
            raise FaultError("No glass detected on the scale")
        self.scale.tare()                                     # zero the glass
        return State.DISPENSE

    def _s_dispense(self):
        pump, target = self.recipe[self.channel]
        print(f"[S{4 + self.channel}] {pump}: {target} g")
        self._pour(pump, target)
        self.channel += 1
        return State.DISPENSE if self.channel < len(self.recipe) else State.VERIFY_TOTAL

    def _s_verify_total(self):
        total_target = sum(t for _, t in self.recipe)
        total_actual = sum(a for _, _, a in self.results)
        err = 100.0 * (total_actual - total_target) / total_target
        print(f"Total: {total_actual:.1f} g vs {total_target} g target ({err:+.1f}%)")
        if abs(err) > VERIFY_FAULT_PCT:
            raise FaultError(f"Total {total_actual:.1f} g is {err:+.1f}% from target")
        if abs(err) > VERIFY_TOL_PCT:
            print(f"WARNING: outside +/-{VERIFY_TOL_PCT}% tolerance")
        return State.SERVE_GLASS

    def _s_serve_glass(self):
        self.arm.grab_and_move_to_end()
        return State.COMPLETE

    def _s_complete(self):
        elapsed = time.monotonic() - self.cycle_start
        print("Results (pump, target g, actual g):")
        for pump, target, actual in self.results:
            print(f"  {pump}: {target} -> {actual:.1f}  ({actual - target:+.1f} g)")
        verdict = "OK" if elapsed <= CYCLE_TARGET_S else f"OVER the {CYCLE_TARGET_S:.0f} s target"
        print(f"Cycle time: {elapsed:.1f} s ({verdict})")
        self.scale.tare()                                     # platform is empty again
        return None if self.once else State.IDLE

    def _s_fault(self):
        self.pumps.stop_all()
        print(f"*** FAULT: {self.fault_msg}")
        if self.once:
            sys.exit(1)
        print("Pumps off. Clear the problem, then press a keypad key (1-3) to reset.")
        while self.keypad.poll() is None:
            time.sleep(0.01)
        return State.INITIALIZE

    # Closed-loop pour
    def _pour(self, pump, target_g):
        lead = W_LEAD_G.get(pump, 1.0)
        start = self.scale.get_grams()
        t0 = time.monotonic()
        self.pumps.on(pump)
        try:
            while True:
                net = self.scale.get_grams() - start
                print(f"  {pump}: {net:6.1f} / {target_g} g", end="\r")
                if net >= target_g - lead:
                    break                                     # cutoff
                waited = time.monotonic() - t0
                if waited > NO_FLOW_S and net < NO_FLOW_MIN_G:
                    raise FaultError(f"{pump}: no flow (primed? reservoir? tubing?)")
                if waited > POUR_TIMEOUT_S:
                    raise FaultError(f"{pump}: timeout before reaching {target_g} g")
        finally:
            self.pumps.off(pump)                              # always off, even on a fault
        time.sleep(DWELL_S)                                   # settle
        actual = self.scale.get_grams() - start
        print(f"  {pump}: {actual:6.1f} / {target_g} g  (settled)")
        if actual > target_g * (1 + OVERFILL_PCT / 100.0) + 3.0:
            raise FaultError(f"{pump}: overfill ({actual:.1f} g vs {target_g} g)")
        self.results.append((pump, target_g, actual))


# Entry Point
def main():
    ap = argparse.ArgumentParser(description="RDDS main firmware")
    ap.add_argument("--sim", action="store_true", help="simulate scale, pumps and keypad (no hardware)")
    ap.add_argument("--key", default="1", help="recipe key to run in --sim mode (1, 2 or 3)")
    ap.add_argument("--sim-rate", type=float, default=5.0, help="simulated pour rate, g/s per pump")
    args = ap.parse_args()

    if args.sim:
        pumps = PumpController(device_factory=_FakeDev)
        app = DrinkDispenser(_SimKeypad([args.key]), pumps, _SimScale(pumps, args.sim_rate),
                             Arm(), once=True)
    else:
        from arm import XArm                                  # needs: pip install xarm, and poses taught
        pumps = PumpController()
        app = DrinkDispenser(Keypad(), pumps, Scale(), XArm())
    app.run()


if __name__ == "__main__":
    main()
