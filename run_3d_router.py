"""
Entry point for the 3D Optical Router (router3d).
"""

import sys
from pathlib import Path

# Add src to python path
src_dir = Path(__file__).resolve().parent / "src"
if str(src_dir) not in sys.path:
    sys.path.insert(0, str(src_dir))

from router3d.config.config import configs
from router3d.config.parser import Parser
from router3d.main import PICRoute


def main():
    argp = Parser()
    args, opts = argp.parser.parse_known_args()
    configs.load(args.config, recursive=False)
    configs.update(opts)

    router = PICRoute(args.benchmark, configs)
    router.run()


if __name__ == "__main__":
    main()
