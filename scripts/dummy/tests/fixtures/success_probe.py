#!/usr/bin/env python3
"""Minimal successful JSON probe fixture for adapter tests."""

import argparse
import json
from pathlib import Path


parser = argparse.ArgumentParser()
parser.add_argument("--report", type=Path, required=True)
arguments = parser.parse_args()
arguments.report.write_text(json.dumps({"observed": True}) + "\n")
