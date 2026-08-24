#!/usr/bin/env python3
"""Create Shields.io endpoint JSON files for ROS apt package availability."""

from __future__ import annotations

import gzip
import json
import os
import sys
from pathlib import Path
from urllib.error import URLError
from urllib.request import urlopen


PACKAGE = "unbag"
REPOSITORY = os.environ.get("ROS_APT_REPOSITORY", "http://packages.ros.org/ros2/ubuntu")
DISTROS = {
    "humble": "jammy",
    "jazzy": "noble",
    "kilted": "noble",
    "lyrical": "noble",
    # Update this codename when the ROS Rolling apt repository moves to a new Ubuntu release.
    "rolling": "noble",
}
OUTPUT_DIRECTORY = Path(os.environ.get("APT_BADGE_OUTPUT_DIRECTORY", ".github/badges/apt-versions"))


def package_version(distro: str, ubuntu_codename: str) -> str | None:
    """Return the current amd64 apt version for a ROS distribution, if present."""
    url = f"{REPOSITORY}/dists/{ubuntu_codename}/main/binary-amd64/Packages.gz"
    target_package = f"ros-{distro}-{PACKAGE}"

    try:
        with urlopen(url, timeout=30) as response:  # nosec B310: configured ROS apt index endpoint
            contents = gzip.decompress(response.read()).decode("utf-8")
    except (OSError, URLError) as error:
        print(f"Could not query {url}: {error}", file=sys.stderr)
        return None

    for stanza in contents.split("\n\n"):
        fields = dict(
            line.split(": ", 1)
            for line in stanza.splitlines()
            if ": " in line and not line.startswith((" ", "\t"))
        )
        if fields.get("Package") == target_package:
            return fields.get("Version")
    return None


def display_version(apt_version: str | None) -> str | None:
    """Remove an optional Debian epoch and packaging revision from an apt version."""
    if apt_version is None:
        return None
    return apt_version.rsplit(":", 1)[-1].split("-", 1)[0]

def write_badge(distro: str, version: str | None) -> None:
    """Write an endpoint schema understood by shields.io."""
    badge = {
        "schemaVersion": 1,
        "label": f"apt · ROS 2 {distro.title()}",
        "message": display_version(version) or "unavailable",
        "color": "0a7d2c" if version else "e05d44",
        "cacheSeconds": 3600,
    }
    destination = OUTPUT_DIRECTORY / f"{distro}.json"
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(badge, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    for distro, ubuntu_codename in DISTROS.items():
        version = package_version(distro, ubuntu_codename)
        print(f"{distro}: {version or 'unavailable'}")
        write_badge(distro, version)


if __name__ == "__main__":
    main()
