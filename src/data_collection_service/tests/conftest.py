import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
TOOLS = ROOT / "tools"
for p in (str(SRC), str(TOOLS)):
    if p not in sys.path:
        sys.path.insert(0, p)
