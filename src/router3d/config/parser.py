import argparse
from pathlib import Path

DEFAULT_BENCHMARK = str(
    Path(__file__).resolve().parents[3]
    / "external/LiDAR/src/picroute/benchmarks/clements_8x8/clements_8x8.yml"
)
DEFAULT_CONFIG = str(
    Path(__file__).resolve().parent / "router3d_default.yml"
)


class Parser(object):
    def __init__(self):
        self.parser = argparse.ArgumentParser()
        self.parser.add_argument(
            "--benchmark",
            default=DEFAULT_BENCHMARK,
            help="path to benchmark",
        )
        self.parser.add_argument(
            "--config",
            default=DEFAULT_CONFIG,
            metavar="File",
            help="the router config",
        )
