import json

class Config():
    def __init__(self, file="/home/dsa/Robotics/config.json"):
        self._file = file
        self._config = {}
        self.load_config()

    def load_config(self):
        with open(self._file, "r") as f:
            self._config = json.load(f)

    def save_config(self):
        with open(self._file, "w") as f:
            json.dump(self._config, f, indent=4)

    def get_value(self, key, default=None):
        return self._config.get(key, default)

    def set_value(self, key, value):
        self._config[key] = value
        return