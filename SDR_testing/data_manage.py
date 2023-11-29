import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import os

print(os.getcwd() + "###################################")
# path = "C:\Users\caspe\OneDrive - Aalborg Universitet\GIT\P5-EIT-AAU-512\SDR_testing"




date_time ="Steering_angles " + str(datetime.now())
# # Load data from files
# # f = open(r"C:\Users\caspe\OneDrive - Aalborg Universitet\GIT\P5-EIT-AAU-512\SDR_testing\Steering_angles.txt", "r")
# # f = open(r"C:\Users\caspe\OneDrive - Aalborg Universitet\GIT\P5-EIT-AAU-512\SDR_testing\Amplitude.txt", "r")


# # steer_angle = np.loadtxt("Steering angles.txt", delimiter=", ")
# # Amplitude = np.loadtxt("Amplitude.txt", delimiter=", ")

# # Plotting
# plt.plot(steer_angle, Amplitude, marker='o')
# plt.xlabel('Steer Angle')
# plt.ylabel('Amplitude')
# plt.title('Steer Angle vs. Amplitude')
# plt.savefig('output_plot.png')
print(date_time)
path = ("C:\Users\caspe\OneDrive - Aalborg Universitet\GIT\P5-EIT-AAU-512\SDR_testing\data_folder\myfile.txt" + str(date_time))
print(path)
f = open(r"C:\Users\caspe\OneDrive - Aalborg Universitet\GIT\P5-EIT-AAU-512\SDR_testing\data_folder\myfile.txt", "x")
f.write