import os
import json
import time
import argparse
import numpy as np
from typing import List, Dict, Any

from rapid_hand_control.servo_driver.rapid_driver import RapidDriver


def format_jointmap(jointmap: List[Any], indent_level: int = 8, items_per_line: int = 4) -> str:
    """Format joint map into compact multi-line JSON-like string."""
    indent = " " * indent_level
    inner_strs = [json.dumps(item, separators=(",", ":")) for item in jointmap]

    lines = []
    for i in range(0, len(inner_strs), items_per_line):
        chunk = inner_strs[i: i + items_per_line]
        lines.append(indent + ", ".join(chunk))

    return "[\n" + ",\n".join(lines) + "\n" + " " * (indent_level - 4) + "]"


def format_list(lst: List[float], indent_level: int = 8, items_per_line: int = 4) -> str:
    """Format a list of numbers for pretty-printing in aligned JSON-style."""
    lines = []
    indent = " " * indent_level
    for i in range(0, len(lst), items_per_line):
        chunk = lst[i: i + items_per_line]
        line = indent + json.dumps(chunk)[1:-1]  # remove surrounding brackets
        lines.append(line)
    return "[\n" + ",\n".join(lines) + "\n" + " " * (indent_level - 4) + "]"


def dump_custom_json(filepath: str, data: Dict[str, Any]) -> None:
    """Write formatted controller parameters to a JSON file."""
    with open(filepath, "w", encoding="utf-8") as f:
        f.write('{\n    "init": {\n')
        keys = ["jointmap", "kP", "kI", "kD", "curr_lim", "init_pos"]

        for i, key in enumerate(keys):
            f.write(f'        "{key}" : ')
            if key == "jointmap":
                f.write(format_jointmap(data["init"][key]))
            else:
                values = data["init"][key]
                if key == "init_pos":
                    values = [round(float(v), 2) for v in values]
                f.write(format_list(values))
            if i != len(keys) - 1:
                f.write(",\n")
            else:
                f.write("\n")
        f.write("    }\n}")


def set_initial_position(motor_port: str) -> None:
    """Query current motor positions and save them into JSON config."""
    config_path = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..", "control", "rapid_hand_control", "servo_driver", "rapid_full.json"
        )
    )

    with open(config_path, "r", encoding="utf-8") as file:
        config_data = json.load(file)

    driver = RapidDriver(port=motor_port, init_set=False)
    current_positions = driver.dxl_client.read_pos()
    config_data["init"]["init_pos"] = [float(p) for p in current_positions]

    dump_custom_json(config_path, config_data)
    print(f"Initial positions saved to: {config_path}")


def run_zero_position_test(motor_port: str, duration: int = 10) -> None:
    """Send zero position to all actuators for testing."""
    driver = RapidDriver(port=motor_port, init_set=True)
    for t in range(duration):
        driver.set_pos(np.zeros(20))
        print(f"[Test] Sent zero position frame {t + 1}/{duration}")
        time.sleep(1)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RapidDriver initialization tool")
    parser.add_argument(
        "-p", "--motor_port",
        type=str,
        default="/dev/ttyUSB0",
        help="Serial port connected to motor controller (default: /dev/ttyUSB0)"
    )
    parser.add_argument(
        "--test",
        action="store_true",
        help="Run test to send zero position commands to motors"
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.test:
        run_zero_position_test(args.motor_port)
    else:
        set_initial_position(args.motor_port)


if __name__ == "__main__":
    main()
