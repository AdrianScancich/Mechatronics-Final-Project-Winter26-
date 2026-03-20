from serial import Serial
from time import sleep
from matplotlib import pyplot as plt
from datetime import datetime
import csv

param = False
skipfile = False
delay = 0.05  # Delay in seconds
done = False
state = 0
running = True
prm = ""
connection = False
testcounter = 0
stateestimatorEnabled = False

while not connection:
    try:
        ser = Serial('COM10', 115200, timeout=180)
        print("Serial port opened successfully")
        connection = True
    except:
        print("Failed to open serial port COM10, attempting to reconnect in 15 seconds...")
        running = False
        sleep(15)  # Wait for 15 seconds before trying again
    
# Initialize Parameters to send to Romi
#t_span = 0
Kp = 0
Ki = 0  
Kd = 0
Setpoint = 0

Title = ("+---------------------------------------------------------+\r\n"
         "|                  Romi Tuning Interface                  |\r\n"
         "+---------------------------------------------------------+\r\n"
         "|   R - Run step response with digital parameters         |\r\n"
         "|   L - Run line follower with digital parameters [SE]    |\r\n"
         "|   C - Change digital parameters                         |\r\n"
         "|   F - Change target folder                              |\r\n"
         "|   M - Manually input parameters                         |\r\n"
         "|   N - Run step response with manually input parameters  |\r\n"
         "|   T - Run line follower with manually input parameters  |\r\n"
         "|   S - Enable/Disable State Estimator                    |\r\n"
         "+---------------------------------------------------------+")

Target_Folder = "C:\\Users\\Admin\\OneDrive - Cal Poly\\ME 405\\ME 405 Lab\\Lab 6\\Testing Data\\"
Test_Parameters_File = f"{Target_Folder}Test_Parameters.csv"

def set_gains(Kp, Ki, Kd):
    # Send parameters to Romi
    # GAINS
    ser.write("k".encode())  # "k" - navigate to gains menu
    sleep(delay)  # Short delay to ensure the command is processed before sending parameters
    ser.write(str(Kp).encode())
    sleep(delay)
    ser.write("\n".encode())  # Enter key to move to next parameter
    sleep(delay)
    ser.write(str(Ki).encode())
    sleep(delay)
    ser.write("\n".encode())
    sleep(delay)
    ser.write(str(Kd).encode())
    sleep(delay)
    ser.write("\n".encode())
    sleep(delay)

def set_lf_gains(LFKp, LFKi, LFKd):
    # Send parameters to Romi
    # GAINS
    ser.write("p".encode())  # "p" - navigate to line follower gains menu
    sleep(delay)  # Short delay to ensure the command is processed before sending parameters
    ser.write(str(LFKp).encode())
    sleep(delay)
    ser.write("\n".encode())  # Enter key to move to next parameter
    sleep(delay)
    ser.write(str(LFKi).encode())
    sleep(delay)
    ser.write("\n".encode())
    sleep(delay)
    ser.write(str(LFKd).encode())
    sleep(delay)
    ser.write("\n".encode())
    sleep(delay)

def set_setpoint(Setpoint):
    # SETPOINT
    ser.write("s".encode()) # "s" - navigate to setpoint menu
    sleep(delay)
    ser.write(str(Setpoint).encode())
    sleep(delay)
    ser.write("\n".encode())
    sleep(delay)

def run_test():
    # START
    ser.reset_input_buffer()
    ser.write("g".encode()) # "g" - start the motor task with the new parameters
    sleep(1.2)  # Wait for the motor task to run for 2.5 seconds

def run_line_follower():
    # START
    ser.reset_input_buffer()
    ser.write("l".encode()) # "l" - start the line follower task with the new parameters
    sleep(5)  # Wait for the line follower task to run for 2.5 seconds
    ser.reset_input_buffer()
    ser.write("l".encode()) # "l" - stop the line follower after 5 seconds
    sleep(1)  # S  hort delay to data is fully transmitted from Romi before attempting to read it

def collect_data(testcounter):
    
    # COLLECT DATA
    data = ser.read_until(b"Data done\r\n").decode()  # Read data from the serial buffer until the "Data done" line is encountered, and decode it to a string
    #data = ser.read(ser.in_waiting).decode()  # Read all available data from the serial buffer
    #print("Data received from Romi:")

    figcounter = 1
    stop = False

    # Process data
    header1 = None
    header2 = None
    header3 = None
    header4 = None
    header5 = None
    header6 = None
    header1idx = 100000               #Initialize header index to a large number to ensure it is set when the first header line is encountered
    header2idx = 100000
    header3idx = 100000
    statevectoridx = 100000
    header4idx = 100000
    inputvectoridx = 100000
    header5idx = 100000
    outputvectoridx = 100000
    header6idx = 100000
    linefollowParamidx = 100000
    datalines = data.splitlines()
    xdata1 = []                     #X data list 
    ydata1 = []                     #Y data list
    xdata2 = []                     #X data list 
    ydata2 = []                     #Y data list
    xdata3 = []                     #X data list 
    ydata3 = []                     #Y data list
    xdata4 = []                     #X data list
    ydata4 = []                     #Y data list
    ydata5 = []                     #Y data list
    ydata6 = []                     #Y data list
    ydata7 = []                     #Y data list    
    ydata8 = []                     #Y data list
    ydata9 = []                     #Y data list
    ydata10 = []                    #Y data list
    ydata11 = []                    #Y data list
    ydata12 = []                    #Y data list
    ydata13 = []                    #Y data list
    ydata14 = []                    #Y data list
    ydata15 = []                    #Y data list
    ydata16 = []                    #Y data list
    ydata17 = []                    #Y data list

    xlabel1 = ""                    #X-axis label
    ylabel1 = ""                    #Y-axis label
    extralabel1 = ""                #Additional label for first plot (e.g. Left motor)
    xlabel2 = ""                    #X-axis label
    ylabel2 = ""                    #Y-axis label
    extralabel2 = ""                #Additional label for second plot (e.g. Right motor)
    xlabel3 = ""                    #X-axis label
    ylabel3 = ""                    #Y-axis label
    extralabel3 = ""                #Additional label for third plot (e.g. Third motor)
    xlabel4 = ""                    #X-axis label
    #ylabel4 = ""                    #Y-axis label
    #extralabel4 = ""                #Additional label for fourth plot (e.g. Fourth motor)
    #xlabel5 = ""                    #X-axis label
    #ylabel5 = ""                    #Y-axis label
    #extralabel5 = ""                #Additional label for fifth plot (e.g. Fifth motor)
    #xlabel6 = ""                    #X-axis label
    #ylabel6 = ""                    #Y-axis label
    #extralabel6 = ""                #Additional label for sixth plot (e.g. Sixth motor)
    #xlabel7 = ""                    #X-axis label
    #ylabel7 = ""                    #Y-axis label
    #extralabel7 = ""                #Additional label for seventh plot (e.g. Seventh motor)
    #xlabel8 = ""                    #X-axis label
    #ylabel8 = ""                    #Y-axis label
    #extralabel8 = ""                #Additional label for eighth plot (e.g. Eighth motor)
    #extralabel9 = ""                #Additional label for ninth plot
    #extralabel10 = ""               #Additional label for tenth plot
    Parameters = []                 #Parameters list
    Vector_Labels = []              #State vector labels list
    run = True                      #Flag used to indicate when to stop processing data

    #print(data)

    for i, data in enumerate(datalines):
        if not data == "Data done":
            #print (f"Line {i}: {data}")  # Print each line of data with its line number for debugging
            if i >= 0:               # Skip the first 0 lines of data 
                #if i == 1:
                #    print(len(datalines))
                if i <= 3:           # Log gains and setpoint parameters from the first 4 lines
                    Parameters.append(data.strip().split(','))  #Extract parameters
    
                if i > 3 and not header1 and data.strip().split(',')[0]:  # Identify the first header line (which contains alphabetic characters)
                    header1 = data.strip().split(',')
                    header1idx = i
                    xlabel1 = header1[0]          #Set x-axis label
                    ylabel1 = header1[1]          #Set y-axis label
                    try:
                        extralabel1 = header1[2]     #Set extralabel if it exists
                    except IndexError:
                        extralabel1 = ""

                if i > header1idx and i < header2idx:  # Collect data lines between the first and second header lines
                    if data == "":  
                        header2idx = i + 1 # Set the index for the second header line to be the line immediately following the empty line 
                    else:
                        #print(f"Line {i}: {data}")
                        dataline = data.strip().split(',') 
                        xval = float(dataline[0])
                        yval = float(dataline[1])
                        ydata1.append(yval)      #Convert y value to float
                        xdata1.append(xval)      #Convert x value to float
                
                if i == header2idx:  # If we encounter the second header line, set the flag to indicate that we are now collecting data after it
                    if not data.strip() == "":  # If the line is empty, skip it and continue to the next line
                        header2 = data.strip().split(',')
                        xlabel2 = header2[0]          #Set x-axis label
                        ylabel2 = header2[1]          #Set y-axis label
                        try:
                            extralabel2 = header2[2]     #Set extralabel if it exists
                        except IndexError:
                            extralabel2 = ""

                if i > header2idx and i < linefollowParamidx:  # Collect data lines after the second header line
                    if data == "":  
                        linefollowParamidx = i + 1 # Set the index for the line follower parameters line to be the line immediately following the empty line
                        header3idx = i + 5 # Set the index for the third header line to be the line immediately following the empty line
                    else:
                        dataline = data.strip().split(',') 
                        xval = float(dataline[0])
                        yval = float(dataline[1])
                        ydata2.append(yval)      #Convert y value to float
                        xdata2.append(xval)      #Convert x value to float

                if i >= linefollowParamidx and i < header3idx:  # If we encounter the line follower parameters line, extract the line follower parameters
                    Parameters.append(data.strip().split(','))  #Extract parameters

                if i == header3idx:  # If we encounter the third header line, set the flag to indicate that we are now collecting data after it
                    if not data.strip() == "":  # If the line is empty, skip it and continue to the next line
                        header3 = data.strip().split(',')
                        xlabel3 = header3[0]          #Set x-axis label
                        ylabel3 = header3[1]          #Set y-axis label
                        try:
                            extralabel3 = header3[2]     #Set extralabel if it exists
                        except IndexError:
                            extralabel3 = ""

                if i > header3idx and i < statevectoridx:  # Collect data lines after the third header line
                    if data == "":  
                        statevectoridx = i + 1 # Set the index for the state vector line to be the line immediately following the empty line
                        header4idx = i + 2 # Set the index for the fourth header line to be the line immediately following the empty line
                    else:
                        dataline = data.strip().split(',') 
                        xval = float(dataline[0])
                        yval = float(dataline[1])
                        ydata3.append(yval)      #Convert y value to float
                        xdata3.append(xval)      #Convert x value to float

                if i == statevectoridx:  # If we encounter the state vector line, extract the state vector parameters
                    Vector_Labels.append(data.strip().split(','))  #Extract parameters

                if i == header4idx:  # If we encounter the fourth header line, set the flag to indicate that we are now collecting data after it
                    if not data.strip() == "":  # If the line is empty, skip it and continue to the next line
                        header4 = data.strip().split(',')
                        xlabel4 = header4[0]          #Set x-axis label
                        header4.pop(0) # Remove x-axis label from header list
                
                if i > header4idx and i < inputvectoridx:  # Collect data lines after the third header line
                    if data == "":  
                        inputvectoridx = i + 1 # Set the index for the input vector line to be the line immediately following the empty line
                        header5idx = i + 2 # Set the index for the fifth header line to be the line immediately following the empty line
                    else:
                        dataline = data.strip().split(',') 
                        xval = float(dataline[0])
                        yval = float(dataline[1])
                        yval1 = float(dataline[2])
                        yval2 = float(dataline[3])
                        yval3 = float(dataline[4])
                        ydata4.append(yval)      #Convert y value to float
                        ydata5.append(yval1)     #Convert y value to float
                        ydata6.append(yval2)     #Convert y value to float
                        ydata7.append(yval3)     #Convert y value to float
                        xdata4.append(xval)      #Convert x value to float
                
                if i == inputvectoridx:  # If we encounter the input vector line, extract the input vector parameters
                    Vector_Labels.append(data.strip().split(','))  #Extract parameters

                if i == header5idx:  # If we encounter the fifth header line, set the flag to indicate that we are now collecting data after it
                    if not data.strip() == "":  # If the line is empty, skip it and continue to the next line
                        header5 = data.strip().split(',')
                
                if i > header5idx and i < outputvectoridx:  # Collect data lines after the third header line
                    if data == "":  
                        outputvectoridx = i + 1 # Set the index for the output vector line to be the line immediately following the empty line
                        header6idx = i + 2 # Set the index for the sixth header line to be the line immediately following the empty line
                    else:
                        dataline = data.strip().split(',') 
                        yval = float(dataline[0])
                        yval1 = float(dataline[1])
                        yval2 = float(dataline[2])
                        yval3 = float(dataline[3])
                        yval4 = float(dataline[4])
                        yval5 = float(dataline[5])
                        ydata8.append(yval)      #Convert y value to float
                        ydata9.append(yval1)     #Convert y value to float
                        ydata10.append(yval2)     #Convert y value to float
                        ydata11.append(yval3)     #Convert y value to float
                        ydata12.append(yval4)     #Convert y value to float
                        ydata13.append(yval5)     #Convert y value to float
                
                if i == outputvectoridx:  # If we encounter the output vector line, extract the output vector parameters
                    Vector_Labels.append(data.strip().split(','))  #Extract parameters

                if i == header6idx:  # If we encounter the sixth header line, set the flag to indicate that we are now collecting data after it
                    if not data.strip() == "":  # If the line is empty, skip it and continue to the next line
                        header6 = data.strip().split(',')
                
                if i > header6idx:  # Collect data lines after the third header line
                    dataline = data.strip().split(',') 
                    yval = float(dataline[0])
                    yval1 = float(dataline[1])
                    yval2 = float(dataline[2])
                    yval3 = float(dataline[3])
                    ydata14.append(yval)      #Convert y value to float
                    ydata15.append(yval1)     #Convert y value to float
                    ydata16.append(yval2)     #Convert y value to float
                    ydata17.append(yval3)     #Convert y value to float  
    #print(header1)
    #print(header2)
    #print(header3)
    #print(header4)
    #print(header5)
    #print(header6)
    #print(Vector_Labels)
    #print(xdata4)
    plt.figure(num=figcounter)  # Create a new figure for the first test     
    if header3 == None: # If a third header was found, create a combined plot of the first two datasets with the parameters in the title, and the third dataset in a separate figure with its own parameters in the title    
        plt.plot(xdata1, ydata1, label=f"{extralabel1} Kp={Kp}, Ki={Ki}, Kd={Kd}, V={Setpoint}")  # Plot the data with a label for the legend
    else:
        plt.plot(xdata1, ydata1, label=f"Test {testcounter}: {extralabel1} Kp={Kp}, Ki={Ki}, Kd={Kd}, V={Setpoint}")  # Plot the data with a label for the legend
    if not header2 == None:  # If a second header was not found, do not plot the second dataset
        if header3 == None:
            plt.plot(xdata2, ydata2, label=f"{extralabel2} Kp={Kp}, Ki={Ki}, Kd={Kd}, V={Setpoint}")  # Plot the data with a label for the legend
        else:  
            plt.plot(xdata2, ydata2, label=f"Test {testcounter}: {extralabel2} Kp={Kp}, Ki={Ki}, Kd={Kd}, V={Setpoint}")
    if not header4 == None:  # If a fourth header was found, plot the state estimator data in a separate figure with its own parameters in the title
        file_path = f"{Target_Folder}State_Estimator_csv_{Kp}_{Ki}_{Kd}_{Setpoint}_{LFKp}_{LFKi}_{LFKd}.csv"
    elif not header3 == None:
        file_path = f"{Target_Folder}{extralabel3.strip().replace(' ', '_')}_csv_{Kp}_{Ki}_{Kd}_{Setpoint}_{LFKp}_{LFKi}_{LFKd}.csv"
    else:
        file_path = f"{Target_Folder}csv_{Kp}_{Ki}_{Kd}_{Setpoint}.csv"
        
    #plotcounter += 1
    plt.legend(loc='lower right')  # Show the legend in the upper right corner
    plt.xlabel(xlabel1)
    plt.ylabel(ylabel1)
    #plt.show()
    time = datetime.now().strftime("%m-%d-%H-%M-%S")
    plt.savefig(f"{Target_Folder}plot_{Kp}_{Ki}_{Kd}_{Setpoint}.png", bbox_inches='tight', dpi=100)  # Save the plot with a timestamped filename

    if not header3 == None:  # If a third header was found, plot the third dataset
        plt.figure(num=(figcounter + 1))  # Create a new figure for the third dataset
        plt.plot(xdata3, ydata3, label=f"LF_Kp={LFKp}, LF_Ki={LFKi}, LF_Kd={LFKd}, V={Setpoint}")  # Plot the data with a label for the legend
        plt.legend(loc='upper right')  # Show the legend in the upper right corner
        plt.xlabel(xlabel3)
        plt.ylabel(ylabel3)
        #plt.show()
        plt.savefig(f"{Target_Folder}{extralabel3.strip().replace(' ', '_')}_plot_{LFKp}_{LFKi}_{LFKd}_{Setpoint}.png", bbox_inches='tight', dpi=100)  # Save the plot with a timestamped filename

    # Open the file in write mode with newline=''
    with open(file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        # Write all rows at once starting with the parameters and header
        try:
            writer.writerows([["Setpoint"], Parameters[0]])
        except IndexError:
            stop = True
            print("Error: No pamaeters found, check Romi output and try again.")
        if not stop:
            writer.writerows([["Kp"], Parameters[1]]
                            + [["Ki"], Parameters[2]]
                            + [["Kd"], Parameters[3]]
                            + [header1] 
                            + list(zip(xdata1, ydata1)))
            if header2:
                writer.writerows([""] # Blank row to separate the two datasets
                                + [header2] 
                                + list(zip(xdata2, ydata2)))
            
            if header3:
                writer.writerows([""] # Blank row to separate the two datasets
                                + [["LF_Kp"], Parameters[4]]
                                + [["LF_Ki"], Parameters[5]]
                                + [["LF_Kd"], Parameters[6]]
                                + [header3] 
                                + list(zip(xdata3, ydata3)))
            
            if header4:
                writer.writerows([""] # Blank row to separate the two datasets
                                + [["State Estimator Data"]]
                                + [[xlabel4]] 
                                + list(zip(xdata4))
                                + [""]
                                + [[Vector_Labels[0][0]]]
                                + [header4]
                                + list(zip(ydata4, ydata5, ydata6, ydata7))
                                + [""]
                                + [[Vector_Labels[1][0]]]
                                + [header5]
                                + list(zip(ydata8, ydata9, ydata10, ydata11, ydata12, ydata13))
                                + [""]
                                + [[Vector_Labels[2][0]]]
                                + [header6]
                                + list(zip(ydata14, ydata15, ydata16, ydata17)))

while running:

    if state == 0: # Initialization
        # ENTER TEST MODE
        ser.reset_input_buffer()
        ser.write("t".encode())  # "t" - enter test mode
        sleep(delay)  # Short delay to ensure the command is processed before sending parameters
        if ser.in_waiting > 0:
            print("Communication established, initialization successful.")
            if Target_Folder != "":
                print(f"Target folder for data files is {Target_Folder}")
            else:            
                print("No target folder specified, data files will be saved to current working directory.")
            if Test_Parameters_File != "":
                print(f"Parameters initialized from {Test_Parameters_File}")
            else:
                print("No parameters file specified")
            print(Title)
            state = 1

        else:
            print("No response from Romi, check connection and try again.")
            running = False
        

    if state == 1:  # User Input
        Prm = input(">: ")
        if Prm.lower() == "m" or Prm.lower() == "M":
            param = False
            while param == False:
                print("Input parameters to send to Romi:")
                print("Kp, Ki, Kd, Setpoint, LFKp, LFKi, LFKd (comma-separated)")
                Prm = input(">: ")
                Prm_received = Prm.split(',')
                try: 
                    #t_span = float(Prm_received[0])  # Check if the first parameter can be converted to float
                    Kp = float(Prm_received[0])  # Check if the second parameter can be converted to float
                    Ki = float(Prm_received[1])  # Check if the third parameter can be converted to float
                    Kd = float(Prm_received[2])  # Check if the fourth parameter can be converted to float
                    Setpoint = float(Prm_received[3])  # Check if the fifth parameter can be converted to float   
                    LFKp = float(Prm_received[4])  # Check if the sixth parameter can be converted to float
                    LFKi = float(Prm_received[5])  # Check if the seventh parameter can be converted to float
                    LFKd = float(Prm_received[6])  # Check if the eighth parameter can be converted to float
                    param = True  # If all parameters are valid, exit the loop
                except ValueError:
                    print("Invalid input. Please enter numeric values.")
                    continue
                except IndexError:
                    print("Invalid input. Please enter 7 parameters.")
                    continue
            state = 2
            set_gains(Kp, Ki, Kd)  # Send gains to Romi
            set_setpoint(Setpoint)  # Send setpoint to Romi
            set_lf_gains(LFKp, LFKi, LFKd)  # Send LFK gains to Romi
            #run_test()  # Run the test with the extracted parameters
            #collect_data()  # Collect data from Romi and generate plots/files
        elif Prm.lower() == "r" or Prm.lower() == "R":
            plt.figure(num=1)  # Create a new figure for the first test
            plt.clf()
            plt.figure(num=2)  # Create a new figure for the first test
            plt.clf()
            plt.figure(num=1)
            plt.close('all')  
            testcounter = 0
            Testfile = open(f"{Test_Parameters_File}")    #Identify data file
            InputParams = Testfile.readlines()    #Compile data into list
            for idx, line in enumerate(InputParams):
                if idx == 0:  # Skip the header line
                    continue 
                #if figcounter == 0:
                #    plt.figure(num=figcounter)  # Create a new figure for the first test
                #if line.strip() == "":  # Skip empty lines
                #    continue  
                params = line.strip().split(',')  # Split the line into parameters
                Kp = float(params[0])  # Extract Kp
                Ki = float(params[1])  # Extract Ki
                Kd = float(params[2])  # Extract Kd
                Setpoint = float(params[3])  # Extract Setpoint
                
                testcounter += 1
                state = 2
                #Prm = ""
                set_gains(Kp, Ki, Kd)  # Send gains to Romi
                set_setpoint(Setpoint)  # Send setpoint to Romi
                run_test()  # Run the test with the extracted parameters
                collect_data(testcounter)  # Collect data from Romi and generate plots/files
        elif Prm.lower() == "l" or Prm.lower() == "L":
            plt.figure(num=1)  # Create a new figure for the first test
            plt.clf()
            plt.figure(num=2)  # Create a new figure for the first test
            plt.clf()
            plt.figure(num=1)
            plt.close('all')  
            testcounter = 0
            Testfile = open(f"{Test_Parameters_File}")    #Identify data file
            InputParams = Testfile.readlines()    #Compile data into list
            for idx, line in enumerate(InputParams):
                if idx == 0:  # Skip the header line
                    continue 
            
                params = line.strip().split(',')  # Split the line into parameters
                Kp = float(params[0])  # Extract Kp
                Ki = float(params[1])  # Extract Ki
                Kd = float(params[2])  # Extract Kd
                Setpoint = float(params[3])  # Extract Setpoint
                
                try:
                    LFKp = float(params[4])  # Extract Kp
                    LFKi = float(params[5])  # Extract Ki
                    LFKd = float(params[6])  # Extract Kd
                except IndexError:
                    LFKp = 0
                    LFKi = 0
                    LFKd = 0

                testcounter += 1
                state = 2
                #Prm = ""
                set_gains(Kp, Ki, Kd)  # Send gains to Romi
                set_lf_gains(LFKp, LFKi, LFKd)  # Send gains to Romi
                set_setpoint(Setpoint)  # Send setpoint to Romi
                print(f"Running Line Follower Test {testcounter} with parameters Kp={Kp}, Ki={Ki}, Kd={Kd}, LF_Kp={LFKp}, LF_Ki={LFKi}, LF_Kd={LFKd}, at {Setpoint} mm/s")
                stateestimatorEnabled = True  # Enable state estimator for line follower tests by default, as it is required for the line follower to function properly. This can be toggled off in the menu if the user wants to run the line follower without the state estimator, but it will likely not perform well without it.
                ser.write("e".encode())  # "e" - enable state estimator
                run_line_follower()  # Run the line follower with the extracted parameters
                collect_data(testcounter)  # Collect data from Romi and generate plots/files
                stateestimatorEnabled = False  # Disable state estimator after each line follower test to prevent it from affecting subsequent tests that may not use it
                ser.write("e".encode())  # "e" - disable state estimator
        elif Prm.lower() == "c" or Prm.lower() == "C":
            print("Enter new filepath for test parameters:")
            Test_Parameters_File = input(">: ")
            print(f"Filepath updated to {Test_Parameters_File}")
            #Prm = ""
            print(Title)
        elif Prm.lower() == "f" or Prm.lower() == "F":
            print("Enter new filepath for target folder:")
            Target_Folder = input(">: ")
            print(f"Target folder updated to {Target_Folder}")
            #Prm = ""
            print(Title)
        elif Prm.lower() == "n" or Prm.lower() == "N":
            if param == False:
                print("No parameters entered. Please enter parameters before running the test.")
            else:
                print("Running Step Response with Manually Input Parameters")
                plt.figure(num=1)  # Create a new figure for the first test
                plt.clf()
                plt.figure(num=2)  # Create a new figure for the first test
                plt.clf()
                plt.figure(num=1) 
                plt.close('all') 
                state = 2

                run_test()  # Run the test with the extracted parameters
                collect_data()  # Collect data from Romi and generate plots/files
        elif Prm.lower() == "t" or Prm.lower() == "T":
            if param == False:
                print("No parameters entered. Please enter parameters before running the test.")
            else:
                print("Running Line Follower with Manually Input Parameters")
                plt.figure(num=1)  # Create a new figure for the first test
                plt.clf()
                plt.figure(num=2)  # Create a new figure for the first test
                plt.clf()
                plt.figure(num=1)
                plt.close('all')  
                state = 2

                run_line_follower()  # Run the line follower with the extracted parameters
                collect_data()  # Collect data from Romi and generate plots/files
        elif Prm.lower() == "s" or Prm.lower() == "S":
            stateestimatorEnabled = not stateestimatorEnabled  # Toggle the state estimator enabled flag
            if stateestimatorEnabled:
                ser.write("e".encode())  # "e" - enable state estimator
                sleep(delay)  # Short delay to ensure the command is processed before sending parameters
                print("State Estimator Enabled")
            else:
                ser.write("e".encode())  # "e" - disable state estimator
                sleep(delay)  # Short delay to ensure the command is processed before sending parameters
                print("State Estimator Disabled")
        else:
            print("Invalid input.")

    if state == 2:  # Test Running
        print(f"Test complete. Datafiles saved to {Target_Folder}.")
        print(Title)
        Prm = ""
        state = 1