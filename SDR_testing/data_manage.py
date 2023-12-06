import os
from matplotlib import pyplot as plt
import numpy as np
from functools import reduce

current_folder = os.path.realpath(__file__).rstrip("data_manage.py")
text_file_position = os.path.join(current_folder, "data_folder")

print("Current working directory:", current_folder)
print("Text file position:", text_file_position)
biggest_numb = 0
newest_file =""
for path, subdirs, files in os.walk(text_file_position):
    print("Walking through:", path)
    for name in files:
        # print("Found file:", os.path.join(path, name))
        new_filename = name.split("_", 2)[2].rstrip(".txt")
        big_numb = new_filename.replace(" ", "")
        if int(big_numb) > biggest_numb:
            biggest_numb = int(big_numb)
            newest_file = text_file_position + "\\" + name

print(biggest_numb)
print(newest_file)
f = open(newest_file, 'r')
filecontaint = f.read()
# print(filecontaint)

numbers_a = []
numbers_s = []
with open(newest_file, 'r') as file:
    # Iterate over each line in the file
    count = 1
    for line in file:
        
        # Process each line as needed
        if count == 6:
            word = ""  # Initialize an empty string to store the number
            numbers_a = []  # List to store extracted numbers
            for character in line:
                if character == ",":
                    print(word)
                    numbers_a.append(float(word))  # Convert the extracted word to a float and append to the list
                    word = ""  # Reset the word for the next number
                else:
                    word += character  # Concatenate the character to the current word
                
        if count == 9:
            word = ""  # Initialize an empty string to store the number
            numbers_s = []  # List to store extracted numbers
            for character in line:
                if character == ",":
                    print(word)
                    numbers_s.append(float(word))  # Convert the extracted word to a float and append to the list
                    word = ""  # Reset the word for the next number
                else:
                    word += character  # Concatenate the character to the current word
        count += 1
print("###################")
print(len(numbers_a))
print(numbers_a)
print(len(numbers_s))
print(numbers_s)



# plt.plot(numbers_s,numbers_a)
# plt.title("dick shit")
plt.plot(numbers_a, numbers_s, marker='o', linestyle='', color='b')

# Adding labels and title
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Plot of X and Y')

# Display the plot
plt.show()

