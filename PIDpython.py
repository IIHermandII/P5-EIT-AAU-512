import matplotlib.pyplot as plt

# Constants
target = 90
initial_valueP = 0
initial_valueI = 0
initial_valueD = 0

K_p = 0.4  # Proportional gain (adjust as needed)
K_i = 0.4
K_d = 0.2

time = [0]

P_values = [0]
I_values = [0]
D_values = [0]

lastErrorD = 0
lastErrorI = 0
# Simulate control loop iterations

num_iterations = 500
for t in range(1, num_iterations):
    # Simulate error (replace with actual error calculation)
    errorP = target - initial_valueP
    errorI = target - initial_valueI
    errorD = target - initial_valueD

    P = K_p * errorP

    I = K_i * (errorI + lastErrorI)
    lastErrorI = errorI

    D = errorD - lastErrorD
    lastErrorD = K_d * errorD

    initial_valueP += P 
    initial_valueI += I
    initial_valueD += D

    # Append values to lists
    time.append(t) #t
    P_values.append(initial_valueP)
    D_values.append(initial_valueD)
    I_values.append(initial_valueI)

# Create plots for P(t), I(t), and D(t) components
plt.figure(figsize=(12, 6))
plt.plot(time, P_values, label="P(t)", color='blue')
plt.plot(time, I_values, label="I(t)", color='green')
plt.plot(time, D_values, label="D(t)", color='red')
# Set labels and title
plt.xlabel("Time")
plt.ylabel("Value")
plt.legend()
plt.title("P(t), Components Over Time")

# plt.ylim(0, 100)  # Set the y-axis limits to 0-100
# plt.xlim(0, num_iterations)  # Set the x-axis limits to the full range
plt.ylim(0, 120)  # Set the y-axis limits to 0-50
plt.xlim(0, 50)  # Set the x-axis limits to 0-20

# Show the plot
plt.grid(True)
plt.show()
