import sys
import os
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR', str(Path(__file__).resolve().parent/'outputs'/'.matplotlib'))
sys.path.insert(0,str(Path(__file__).resolve().parent/'src'))
from uav_cooperation.application.cli import main
if __name__ == '__main__':
    raise SystemExit(main())
