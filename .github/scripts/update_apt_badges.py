#!/usr/bin/env python3
"""Create Shields.io endpoint JSON files for ROS apt package availability."""

from __future__ import annotations

import gzip
import json
import os
import subprocess
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


def repository_version() -> str | None:
    """Return the newest semantic version represented by a repository tag."""
    try:
        result = subprocess.run(
            ["git", "tag", "--sort=-version:refname"],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError) as error:
        print(f"Could not read repository tags: {error}", file=sys.stderr)
        return None

    for tag in result.stdout.splitlines():
        version = tag.removeprefix("v")
        if tag.startswith("v") and version and all(part.isdigit() for part in version.split(".")):
            return version
    return None


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


def badge_color(apt_version: str | None, tag_version: str | None) -> str:
    """Select green for matching releases, orange for a version mismatch, red if absent."""
    if apt_version is None:
        return "e05d44"
    return "97ca00" if display_version(apt_version) == tag_version else "orange"


def write_badge(distro: str, version: str | None, tag_version: str | None) -> None:
    """Write an endpoint schema understood by shields.io."""
    badge = {
        "schemaVersion": 1,
        "label": f"{distro.title()}",
        "namedLogo": "ros",
        "logoColor": "white",
        "message": display_version(version) or "unavailable",
        "color": badge_color(version, tag_version),
        "cacheSeconds": 3600,
    }
    destination = OUTPUT_DIRECTORY / f"{distro}.json"
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(badge, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    tag_version = repository_version()
    print(f"repository tag: {tag_version or 'unavailable'}")
    for distro, ubuntu_codename in DISTROS.items():
        version = package_version(distro, ubuntu_codename)
        print(f"{distro}: {version or 'unavailable'}")
        write_badge(distro, version, tag_version)


if __name__ == "__main__":
    main()
