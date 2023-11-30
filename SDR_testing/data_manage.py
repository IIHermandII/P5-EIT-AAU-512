import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import os
from pathlib import Path

# # Get the current script's path
# current_script = os.path.realpath(__file__)

# # Extract the folder path from the script's path
# current_folder = os.path.dirname(current_script)

# # Construct the folder path for the data files
# text_file_position = os.path.join(current_folder, "data_folder")

# # Print the folder path
# print(text_file_position)

# # Construct the file name with date and time
# date_time = datetime.now().strftime("%d %m %Y %H %M %S")
# text_file = os.path.join(text_file_position, f"Steering_angles_{date_time}.txt")

# # Print the full file path
# print(text_file)

# # Create the file
# with open(os.path.normpath(text_file), "w") as f:
#     # Add content to the file if needed
#     f.write("Your content goes here")

current_folder = os.getcwd()
text_file_position = os.path.join(current_folder, "data_folder")

# Create the data_folder if it doesn't exist
if not os.path.exists(text_file_position):
    os.makedirs(text_file_position)

date_time = datetime.now().strftime("%d %m %Y %H %M %S")
text_file = os.path.join(text_file_position, f"Steering_angles_{date_time}.txt")
print(text_file)
# Write content to the file
with open(os.path.normpath(text_file), "w") as f:
    f.write("Your content goes here")

