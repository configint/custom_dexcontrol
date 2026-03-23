# Version is derived from git tags via setuptools-scm
# This file provides a fallback and runtime access to version

try:
    from _version import __version__
except ImportError:
    try:
        from setuptools_scm import get_version
        __version__ = get_version(root=".", relative_to=__file__)
    except (ImportError, LookupError):
        __version__ = "0.0.0+unknown"


def get_version_info() -> dict:
    """Get detailed version information from version_info.txt or git."""
    import os

    info = {
        "version": __version__,
        "branch": "unknown",
        "commit": "unknown",
        "tag": "unknown",
    }

    # Try version_info.txt first (deployed environment without .git/)
    version_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "version_info.txt")
    if os.path.exists(version_file):
        with open(version_file, "r", encoding="utf-8") as f:
            for line in f.read().splitlines():
                if ": " in line:
                    key, value = line.split(": ", 1)
                    if key in info:
                        info[key] = value
        return info

    # Fallback: read from git directly (development environment)
    import subprocess

    git_dir = os.path.dirname(os.path.abspath(__file__))

    try:
        result = subprocess.run(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"],
            cwd=git_dir, capture_output=True, text=True
        )
        if result.returncode == 0:
            info["branch"] = result.stdout.strip()

        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=git_dir, capture_output=True, text=True
        )
        if result.returncode == 0:
            info["commit"] = result.stdout.strip()

        result = subprocess.run(
            ["git", "describe", "--tags", "--exact-match", "HEAD"],
            cwd=git_dir, capture_output=True, text=True
        )
        if result.returncode == 0:
            info["tag"] = result.stdout.strip()
        else:
            info["tag"] = f"v{__version__}"

    except Exception:
        pass

    return info
