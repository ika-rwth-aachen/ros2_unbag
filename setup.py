import subprocess

from setuptools import setup
from setuptools.command.install import install


APT_PACKAGES = [
    'libxcb-cursor0',
    'libxcb-shape0',
    'libxcb-icccm4',
    'libxcb-keysyms1',
    'libxkbcommon-x11-0'
]

class CustomInstall(install):
    def run(self):
        missing = []
        for pkg in APT_PACKAGES:
            res = subprocess.call(['dpkg', '-s', pkg], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if res != 0:
                missing.append(pkg)
        if missing:
            raise RuntimeError(
                f"Missing APT packages: {' '.join(missing)}. "
                "Please install them manually before proceeding: sudo apt install " + ' '.join(missing)
            )
        install.run(self)

setup(
    setup_requires=[],
    cmdclass={'install': CustomInstall}
)