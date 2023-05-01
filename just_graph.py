
# Python code for realtime visualization of charging process
# Using serial input data from Arduino
# Using matplotlib for plotting

import serial # Library for serial communication
import matplotlib.pyplot as plt # Library for plotting
import matplotlib.animation as animation # Library for animation

ser = serial.Serial('COM3', 9600) # Create a serial object with port and baud rate

fig = plt.figure() # Create a figure object
ax = fig.add_subplot(1, 1, 1) # Create an axes object

ambientTemp = [] # List to store ambient temperature
batteryDT = [[], [], [], []] # List to store battery temperature difference of each channel
xdata = [] # List to store x-axis data

def animate(i): # Define a function to animate the plot
  
  global ambientTemp, batteryDT, xdata # Use global variables
  
  line = ser.readline().decode('utf-8') # Read a line from serial and decode it
  
  if line.startswith('Channel'): # If line contains temperature data
    
    data = line.split() # Split the line by spaces
    
    channel = int(data[1]) - 1 # Get the channel number
    
    temp = float(data[4]) # Get the temperature value
    
    if channel == 0: # If channel is 1
      
      ambientTemp.append(temp) # Append temperature to ambient temperature list
      
      xdata.append(i) # Append index to x-axis data
      
    else: # If channel is not 1
      
      batteryDT[channel - 1].append(temp) # Append temperature to battery temperature difference list
      
    ax.clear() # Clear the axes
    
    ax.plot(xdata, ambientTemp, label='Ambient Temperature') # Plot ambient temperature
    
    for j in range(4): # Loop through each channel
      
      ax.plot(xdata, batteryDT[j], label=f'Channel {j + 1} Temperature Difference') # Plot battery temperature difference
      
    ax.legend(loc='upper left') # Add a legend
    ax.set_xlabel('Time') # Set x-axis label
    ax.set_ylabel('Temperature (C)') # Set y-axis label
    ax.set_title('Realtime Visualization of Charging Process') # Set title
    
ani = animation.FuncAnimation(fig, animate, interval=1000) # Create an animation object with figure, function and interval

plt.show() # Show the plot
