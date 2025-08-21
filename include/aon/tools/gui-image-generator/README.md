# Simplified API GUI Image Generator

This tool generates code to display images using PROS's [Simplified Brain Screen C++ API](https://pros.cs.purdue.edu/v5/api/cpp/screen.html).

### How to use

 0. Install the dependencies. Run `pip install -r requirements.txt`.

 1. Edit the images you want to display and store in the `input` folder. Use the following directions:  
  a. Make sure the image is within the 480px by 240px constraint of the display's dimensions.  
  b. Add alpha channel and remove the background to make the output considerably smaller.  
  c. Keep images simple with a small range of colors.  
  d. It's best to hav rows with long chains of pixels with the same color.  
  > The first constraint (part a) is essential, and not meeting it will raise an error. The others help optimize the output and reduce it's file size.

 2. Edit [config.py](./config.py) to include your image in the `INPUT_IMAGES`. Each object in the array must have:  
  a. `function-name` - The function in the output code will be named accordingly. For example, if we have `"function-name": "AONLogo"`, the generated function's name will be `DrawAONLogo`.  
  b. `file-name` - The image's file name. The image MUST be inside `input`  

 3. Run the `main.py` script.


Ta-da! 🎉 You generated an image you can now display with the Simple Screen API. Just import `gui-images.hpp`.