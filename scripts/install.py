import os
import tomllib

with open("pyproject.toml", "rb") as f:
  toml = tomllib.load(f)
robotpy_version = toml["tool"]["robotpy"]["robotpy_version"]
os.system(f'python -m pip install --upgrade robotpy=={ robotpy_version } certifi')
