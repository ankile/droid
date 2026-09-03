"""Per-machine DROID station configuration, kept OUT of the git checkout.

Upstream DROID expects you to edit ``droid/misc/parameters.py`` on every machine
(IPs, robot serial, and the NUC's sudo password). That leaves credentials and
station identity as an uncommitted diff inside a public-repo checkout. Instead,
``parameters.py`` reads them through :func:`load_station_env`:

* explicit ``DROID_*`` environment variables win (tests, one-off overrides);
* otherwise the values come from the station env file, default
  ``~/.config/droid/station.env`` (override the path with
  ``DROID_STATION_ENV_FILE``): one ``KEY=VALUE`` per line, ``#`` comments,
  optional single/double quotes around the value;
* a key present in neither is the upstream blank string.

Unknown keys in the file are an error (a typo must not silently leave a field
blank), and a file that carries ``DROID_SUDO_PASSWORD`` must not be readable by
group/other. No third-party imports: this runs on the NUC's Python 3.6 too.
"""

import os
import stat

STATION_KEYS = (
    "DROID_NUC_IP",
    "DROID_ROBOT_IP",
    "DROID_LAPTOP_IP",
    "DROID_ROBOT_TYPE",  # 'panda' or 'fr3'
    "DROID_ROBOT_SERIAL_NUMBER",
    "DROID_HAND_CAMERA_ID",
    "DROID_VARIED_CAMERA_1_ID",
    "DROID_VARIED_CAMERA_2_ID",
    "DROID_SUDO_PASSWORD",  # NUC only: launches the franka controller via `sudo -S`
    "DROID_UBUNTU_PRO_TOKEN",  # NUC setup only
)
SECRET_KEYS = ("DROID_SUDO_PASSWORD", "DROID_UBUNTU_PRO_TOKEN")
DEFAULT_STATION_ENV_FILE = os.path.join(os.path.expanduser("~"), ".config", "droid", "station.env")


def station_env_path():
    return os.environ.get("DROID_STATION_ENV_FILE", DEFAULT_STATION_ENV_FILE)


def _parse(path):
    values = {}
    with open(path) as fh:
        for lineno, raw in enumerate(fh, 1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if "=" not in line:
                raise ValueError("%s:%d: expected KEY=VALUE, got %r" % (path, lineno, raw.rstrip()))
            key, value = line.split("=", 1)
            key = key.strip()
            value = value.strip()
            if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
                value = value[1:-1]
            if key not in STATION_KEYS:
                raise ValueError(
                    "%s:%d: unknown key %r (allowed: %s)" % (path, lineno, key, ", ".join(STATION_KEYS))
                )
            values[key] = value
    return values


def load_station_env(path=None):
    """Return {key: value} for every STATION_KEY (blank when unset).

    Precedence: explicit environment variable > station env file > "".
    """
    path = path or station_env_path()
    file_values = {}
    if os.path.exists(path):
        file_values = _parse(path)
        if any(file_values.get(k) for k in SECRET_KEYS):
            mode = stat.S_IMODE(os.stat(path).st_mode)
            if mode & (stat.S_IRWXG | stat.S_IRWXO):
                raise PermissionError(
                    "%s holds a secret but is group/other-accessible (mode %o); run: chmod 600 %s"
                    % (path, mode, path)
                )
    values = {}
    for key in STATION_KEYS:
        env = os.environ.get(key)
        values[key] = env if env is not None else file_values.get(key, "")
    return values
