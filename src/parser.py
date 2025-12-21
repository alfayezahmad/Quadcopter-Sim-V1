import os


class MissionParser:
    """Interprets custom DroneScript DSL into stateful mission commands."""

    def __init__(self, filename: str):
        self.commands = []
        self.index = 0
        self.wait_start_time = None
        self._load(filename)

    def _load(self, filename: str):
        if not os.path.exists(filename):
            raise FileNotFoundError(f"Mission file {filename} not found.")
        with open(filename, 'r') as f:
            for line in f:
                line = line.split('#')[0].strip()
                if line:
                    parts = line.split()
                    self.commands.append({
                        "cmd": parts[0].upper(),
                        "val": float(parts[1]) if len(parts) > 1 else 0.0
                    })

    def get_logic(self, current_alt: float, current_time: float):
        if self.index >= len(self.commands):
            return None

        item = self.commands[self.index]

        # Position-based logic (TAKEOFF, MOVE, LAND)
        if item["cmd"] in ["TAKEOFF", "MOVE", "LAND"]:
            if abs(current_alt - item["val"]) < 0.2:
                self.index += 1
                return self.get_logic(current_alt, current_time)
            return item["val"]

        # Time-based logic (WAIT)
        elif item["cmd"] == "WAIT":
            if self.wait_start_time is None:
                self.wait_start_time = current_time
            if (current_time - self.wait_start_time) >= item["val"]:
                self.index += 1
                self.wait_start_time = None
                return self.get_logic(current_alt, current_time)
            return self.commands[self.index - 1]["val"]