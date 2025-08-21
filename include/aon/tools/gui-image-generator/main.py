#!/usr/bin/env python3

"""Takes images in the `input` folder and generates PROS code in `output`.
"""

from PIL import Image
import config
import os


def cd_here():
  """ Sets the working directory to this script's directory.

  Helps run the script from wherever the user's present working directory is.
  This makes running this script a little less error prone.
  """
  file_path = os.path.abspath(__file__)
  file_directory = os.path.dirname(file_path)
  os.chdir(file_directory)


def delete_output():
  """ Delete old output file, if it exists.

  Helps to always have the most up to date version of the output file.
  """
  if os.path.isfile(config.OUTPUT_FILE):
    os.remove(config.OUTPUT_FILE)


def validate_image(path: str) -> bool:
  """ Makes sure that the image is readable and has the appropriate size.

  Not having the correct image format or an image that is too big can create
  errors down the line that could create runtime errors when displaying images
  in the V5 screen. It's best to catch these errors early, and prevent
  generating the code, which can be resource-consuming at times.

  Args:
    path: Path to the image we want to make sure is valid.

  Returns:
    True if image passes all tests. Otherwise raises errors.

  Raises:
    ValueError: Image does not have a valid format.
    ValueError: Image was not found inside `input` directory.
    ValueError: Image dimensions exceed those limited by the configuration file.
  """
  _, extension = os.path.splitext(path)
  if extension[1:] not in config.VALID_IMAGE_FORMATS:
    raise ValueError(
      f"[❌] \"{path}\" is not stored in a valid image format ({extension}).\
      \nCheck https://urlzs.com/jFmNd and change config.py accordingly.")

  if not os.path.isfile(path):
    raise ValueError(
      f"[❌] \"{path}\" is not a recognized image inside the `input` directory.")

  image = Image.open(path).convert("RGBA")
  width, height = image.size

  if width > config.SCREEN_WIDTH:
    raise ValueError(
      f"[❌] \"{path}\" is too wide ({width}px > {config.SCREEN_WIDTH}px)")

  if height > config.SCREEN_HEIGHT:
    raise ValueError(
      f"[❌] \"{path}\" is too tall ({height}px > {config.SCREEN_HEIGHT}px)")

  return True


def extract_image(path: str) -> list:
  """ Takes image path and generates a matrix of its RGBA values.

  Pillow only returns a flat array of RGBA colors, so we must convert it to a
  2D array ourselves in order to use it later.

  Args:
    path: Path to the image we want to process. Should have passed image
      through `validate_image`

  Returns
    2D array of RGBA pixels representing image.
  """
  image = Image.open(path).convert("RGBA")

  flat_array = list(image.getdata())
  columns, rows = image.size

  # Convert 1d array of tuples into a 2d array of tuples.
  matrix = [[flat_array[row*columns + column]
         for column in range(columns)] for row in range(rows)]

  return matrix


def simplify_image(matrix: list) -> list:
  """ Preprocess matrix to remove transparent pixels and apply RLE encoding.

  Removing pixels with alpha less than 0.5 is crucial since we can only display
  RGB values in the V5 brain. Additionally, rows of pixels are simplified to
  an RLE encoding, which prevents repetition by simply specifying how many
  times the color is repeated. This reduces the size of the output file.

  Args:
    matrix: 2D array of RGBA values of an image

  Returns
    2D array of dictionaries with pixel data. Single pixels will only include
    their column and color. Rows of pixels will include color, start
    column, and end column.
  """
  def rgba_to_rgb(x: list) -> list:
    """Removes alpha component from RGBA pixel"""
    return (x[0], x[1], x[2])

  def rgb_matches(x: list, y: list) -> bool:
    """Compares RGB components of two RGBA pixels"""
    return x[0] == y[0] and x[1] == y[1] and x[2] == y[2]

  def int2hexstr(num: list) -> str:
    """Converts an integer into a hex string without the prefix."""
    return str(hex(num))[2:].upper().zfill(2)

  # Apply RLE encoding to RGBA matrix
  length, output = 1, []
  for i, row in enumerate(matrix):
    output.append([])
    j = 0
    while j < len(row) - 1:
      # Remove transparent pixels (pixels with opacity less than 0.5)
      if matrix[i][j][3] < 0.5:
        j += 1
        continue

      # Find sequence of repeated pixels
      length = 1
      while all((
        j + length < len(matrix[i]),
        matrix[i][j + length][3] >= 0.5,
        rgb_matches(matrix[i][j], matrix[i][j + length]),
      )):
        length += 1

      # If the sequence of pixels is longer than 2 pixels, the marginal
      #   benefit of summarizing them in a for loop is greater than the
      #   marginal cost of the space consumed by the loop. Therefore,
      #   if there are only 2 consecutive pixels, they won't be optimized
      #   through the RLE encoding trick.
      if length > 2:
        output[i].append({"start": j, "end": j + length,
                  "color": rgba_to_rgb(matrix[i][j]),
                  })
        j += length

      else:
        output[i].append(
          {"column": j, "color": rgba_to_rgb(matrix[i][j])})
        j += 1

  # Convert colors into HEX that the Simplified Screen API can understand
  for i in range(len(output)):
    for j in range(len(output[i])):
      r, g, b = output[i][j]["color"]
      hexstr_color = f"0x{int2hexstr(r)}{int2hexstr(g)}{int2hexstr(b)}"
      output[i][j].update({"color": hexstr_color})

  return output


def generate_function(function_name: str, simplified_image: list) -> str:
  """ Creates the function for the output code given the processed image.

  I decided to exclude newlines, making code a lot uglier and not comply with
  Google's style, but significantly reducing the file size.

  This function also creates an array of colors in order to reduce the 8
  character hex strings + double quotes into simple array access called `C`
  with 3 digit index at max, making it a 40% reduction of characters in the
  worst cases.

  Args:
    function_name: Name of the function in the output file.
    simplified_image: Image already passed through the `simplify` method.

  Returns:
    String with code for the entire function.

  """

  # Function header
  function = f"inline void Draw{function_name}"
  function += "(unsigned short x, unsigned short y){\n"

  # Will store the code, but will be added to the function after the array C
  body = ""

  # Crucial to the color array optimization. The list is a table of colors, and
  #   the dictionary just returns the color's index in the `colors` list to
  #   reduce lookup time.
  colors, find_colors = [], {}

  for row in range(len(simplified_image)):
    for pixel in simplified_image[row]:

      # Add color to the array and dictionary
      color = pixel["color"]
      if color not in find_colors:
        find_colors[color] = len(colors)
        colors.append(color)

      # If we found an RLE encoded sequence of pixels ...
      if "start" in pixel and "end" in pixel:
        body += f"for(i={pixel['start']};i<{pixel['end']};i++)"
        body += f"DP(C[{find_colors[color]}],i+x,{row}+y);"

      # Otherwise, we only have a single pixel.
      else:
        body += f"DP(C[{find_colors[color]}],{pixel['column']}+x,{row}+y);"

  # Document and add the array of colors to the function.
  function += "  // \"Colors\". Array that stores colors in order to reduce\
repetition.\n"
  function += f"  const std::uint32_t C[] = {{{','.join(colors)}}};\n\n"

  # Add the body
  function += f"  int i = 0;\n  {body}"

  # Finish the function
  function += "\n}\n\n"

  return function


def start_code() -> str:
  """ Generates the base code

  The base code involves imports and making and documenting the helper function
  `DP`.

  Returns:
    String with the beginning of the code that will be generated by the end of
    the program.

  """
  return """#ifndef AON_TOOLS_GUI_IMAGE_GENERATOR_GUI_IMAGES_HPP_
#define AON_TOOLS_GUI_IMAGE_GENERATOR_GUI_IMAGES_HPP_

#include "../../../api.h"  // importing pros

namespace aon {

/**
 * \\brief "DrawPixel". Reduces amount of code needed to draw a pixel.
 *
 * \\details Uses the Simple Screen API to set the pen color to the desired RGB
 *       color and draws the pixel on the desired position.
 */
inline void DP(std::uint32_t color, std::int16_t x, std::int16_t y) {
  pros::screen::set_pen(color);
  pros::screen::draw_pixel(x, y);
}

// === === === === === === === === === === === === === === === === === //
"""


def end_code() -> str:
  """ Returns string with the end of the file.
  """
  return "\n\n}  // namespace aon\n\n\
#endif // AON_TOOLS_GUI_IMAGE_GENERATOR_GUI_IMAGES_HPP_"

def main():
  cd_here()
  delete_output()

  code = start_code()

  for image in config.INPUT_IMAGES:
    path = os.path.join(config.INPUT_DIRECTORY, image["file-name"])
    validate_image(path)
    parsed_image = extract_image(path)
    simplified_image = simplify_image(parsed_image)
    code += generate_function(image["function-name"], simplified_image)

  code += end_code()

  with open(config.OUTPUT_FILE, "x+", encoding="ascii") as fp:
    fp.write(code)

  print("[✔] Done!")


if __name__ == "__main__":
  main()
