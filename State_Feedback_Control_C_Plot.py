import control as ct
import matplotlib.pyplot as plt
import pandas as pd

# Read data from data.txt into a pandas DataFrame
data = pd.read_csv('data.txt', delim_whitespace=True, names=["Time", "Command", "Response", "Setpoint"])

# Target transfer function
s = ct.TransferFunction.s
T = (0.15*s + 0.064)/(s**3 + 1.2*s**2 + 0.48*s + 0.064)

# Ideal step response
t, yout = ct.step_response(100*T)

# Create figure
plt.figure()

# Plot response from C code
plt.subplot(2, 1, 1)
plt.plot(t, yout, label=f"Ideal")
plt.plot(data["Time"], data["Response"], label=f"Response - from C Code")
plt.plot(data["Time"], data["Setpoint"], '--', label=f"Setpoint")
plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.legend()
plt.grid()

# Plot controller output
plt.subplot(2, 1, 2)

plt.plot(data["Time"], data["Command"], label=f"Command - from C Code")

plt.xlabel("Time [s]")
plt.ylabel("F [N]")
plt.legend()
plt.grid()

# Show plots
plt.show()