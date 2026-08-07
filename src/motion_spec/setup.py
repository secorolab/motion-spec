# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""Install external tools required by every motion-spec installation."""

from __future__ import annotations

import shlex
import shutil
import subprocess
from pathlib import Path
from urllib.request import urlretrieve

STST_REPO = "https://github.com/jsnyders/STSTv4.git"
STST_REF = "e6152907c7f82aba7685c4f2630490968ffd6594"
JARS = {
    "ST4-4.3.4.jar": "https://repo1.maven.org/maven2/org/antlr/ST4/4.3.4/ST4-4.3.4.jar",
    "antlr-runtime-3.5.3.jar": (
        "https://repo1.maven.org/maven2/org/antlr/antlr-runtime/3.5.3/antlr-runtime-3.5.3.jar"
    ),
}
DEFAULT_PREFIX = Path.home() / ".local"


def find_stst() -> str | None:
    """Prefer motion-spec's managed STST, falling back to PATH."""
    managed = DEFAULT_PREFIX / "bin" / "stst"
    return str(managed) if managed.is_file() else shutil.which("stst")


def remove_stst(prefix: Path) -> bool:
    """Remove only an STST installation previously managed under PREFIX."""
    launcher = prefix / "bin" / "stst"
    root = prefix / "share" / "motion-spec" / "STSTv4"
    marker = root.parent / ".STSTv4-managed"
    if not (root.exists() or launcher.exists() or marker.exists()):
        return False
    if not marker.is_file():
        raise RuntimeError(f"refusing to clean an unmanaged STST installation under {prefix}")
    shutil.rmtree(root, ignore_errors=True)
    launcher.unlink(missing_ok=True)
    marker.unlink(missing_ok=True)
    try:
        root.parent.rmdir()
    except OSError:
        pass
    return True


def install_stst(prefix: Path) -> Path:
    """Build the pinned STSTv4 and install its launcher under PREFIX/bin."""
    launcher = prefix / "bin" / "stst"
    root = prefix / "share" / "motion-spec" / "STSTv4"
    marker = root.parent / ".STSTv4-managed"
    if launcher.is_file():
        return launcher

    missing = [command for command in ("git", "ant", "java") if shutil.which(command) is None]
    if missing:
        raise RuntimeError(
            f"required command{'s' if len(missing) > 1 else ''} missing: {', '.join(missing)}"
        )

    if not (root / ".git").is_dir():
        root.parent.mkdir(parents=True, exist_ok=True)
        marker.touch()
        subprocess.run(["git", "clone", STST_REPO, str(root)], check=True)
    subprocess.run(["git", "-C", str(root), "fetch", "--tags", "origin"], check=True)
    subprocess.run(["git", "-C", str(root), "checkout", STST_REF], check=True)
    subprocess.run(["ant", "-f", str(root / "build.xml")], check=True)

    lib = root / "lib"
    lib.mkdir(exist_ok=True)
    for filename, url in JARS.items():
        path = lib / filename
        if not path.exists():
            urlretrieve(url, path)

    launcher.parent.mkdir(parents=True, exist_ok=True)
    launcher.write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        f"STST_HOME={shlex.quote(str(root))}\n"
        'CP="$STST_HOME/build/jar/stst.jar:$STST_HOME/lib/ST4-4.3.4.jar:'
        '$STST_HOME/lib/antlr-runtime-3.5.3.jar"\n'
        'exec java -cp "$CP" jjs.stst.STStandaloneTool "$@"\n'
    )
    launcher.chmod(0o755)
    return launcher
