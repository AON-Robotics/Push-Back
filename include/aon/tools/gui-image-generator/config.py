#!/usr/bin/env python3

"""Configuration for this project's `main.py` script
"""

import os

# V5 Screen dimensions. No need to change this
SCREEN_WIDTH = 480
SCREEN_HEIGHT = 240

# Valid image formats. Can be extended, but I think these are enough.
VALID_IMAGE_FORMATS = set(["png", "jpeg", "jpg", "webp", "gif"])

# Where are we getting the images from?
INPUT_DIRECTORY = os.path.join(".", "input")

# This is where all the images to be processed will be specified.
#   ALWAYS include `function-name` and `file-name` in these objects.
INPUT_IMAGES = [
    {
      "function-name": "AONLogo",
      "file-name": "team-logo.png",
    },
    # {
    #   "function-name": "VRCLogo",
    #   "file-name": "vrc-logo.webp",
    # },
]

# This is where the `.hpp` file will be stored at the end.
OUTPUT_FILE = os.path.join(".", "gui-images.hpp")
