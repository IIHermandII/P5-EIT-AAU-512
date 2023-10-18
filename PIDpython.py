import matplotlib.pyplot as plt
# Global variables:
K_p = 0.4  # Proportional gain (adjust as needed)
K_i = 0.6
K_d = 0.2
K_c = 0.3
p_cap = 1
i_cap = 1
d_cap = 1
pid_cap = 20


# Constants
def separatPID():
    print("Separat P, I ,D")
    target = 90
    initial_valueP = 0  
    initial_valueI = 0
    initial_valueD = 0


    time = [0]

    P_values = [0]
    I_values = [0]
    D_values = [0]

    lastErrorP = 0
    lastErrorI = 0
    lastErrorD = 0

    # Simulate control loop iterations
    num_iterations = 100
    for t in range(1, num_iterations):
        # Simulate error (replace with actual error calculation)
        errorP = target - initial_valueP
        errorI = target - initial_valueI
        errorD = target - initial_valueD

        print("--------------------------")
        
        if K_p * errorP < p_cap: 
            P = K_p * errorP
        P = K_p * errorP
        print(str(P) + " p")
        I = K_c * (errorI + (K_i * lastErrorI))
        lastErrorI = errorI
        print(str(I) + " I")
        D = K_c * (errorD - (K_d * lastErrorD))
        lastErrorD = errorD
        print(str(D) + " D")
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
    plt.xlabel("Time")
    plt.ylabel("Value")
    plt.legend()
    plt.title("P(t), I(t), D(t) ")
    # Set the y-axis limits to 0-120 and x-axis limits to 0-50 (adjust as needed)
    plt.ylim(0, 120)
    plt.xlim(0, 50)
    # Show the plot
    plt.grid(True)

def totalPID():
    print("Total PID")
    target = 90
    initial_value = 0  

    time = [0]

    values = [0]

    lastError = 0

    # Simulate control loop iterations
    num_iterations = 100
    for t in range(1, num_iterations):
        # Simulate error (replace with actual error calculation)
        error = target - initial_value

        print("--------------------------")
        P = K_p * error
        print(str(P) + " p")
        I = K_c * (error + (K_i * lastError))
        print(str(I) + " I")
        D = K_c * (error - (K_d * lastError))
        lastError = error
        print(str(D) + " D")
        if P+I+D<pid_cap:
            initial_value += (P + I + D)
        else: 
            initial_value += pid_cap
        
        print(str(initial_value) + "PID")

        # Append values to lists
        time.append(t) #t
        values.append(initial_value)

    # Create plots for P(t), I(t), and D(t) components
    plt.figure(figsize=(12, 6))
    plt.plot(time, values, label="PID(t)", color='blue')
    # Set labels and title
    plt.xlabel("Time")
    plt.ylabel("Value")
    plt.legend()
    plt.title("PID(t) ")
    # Set the y-axis limits to 0-120 and x-axis limits to 0-50 (adjust as needed)
    plt.ylim(0, 120)
    plt.xlim(0, 50)
    # Show the plot
    plt.grid(True)
   
# Call the function to simulate and plot the components
separatPID()
totalPID()

plt.show()
