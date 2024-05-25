#!/Library/Frameworks/Python.framework/Versions/3.9/bin/python3
import serial
#print("PySerial version:", serial.VERSION)

import serial.tools.list_ports

ports = serial.tools.list_ports.comports()
for port, desc, hwid in sorted(ports):
    print("{}: {} [{}]".format(port, desc, hwid))

print("Nom du port série: ", port)

# Configure the serial port, replace '/dev/tty.usbmodem21201' with your Arduino's COM port
ser = serial.Serial(port, 460800)

# Open the file in write mode, which clears any existing data and starts fresh
with open("/Users/mathiasrechsteiner/Desktop/Cours ING 3/Semestre 2/Traitement du Signal/output.txt", "w") as file:
    while True:
        if ser.in_waiting > 0:
            # Decode each line using UTF-8, replace errors to prevent crashes
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if line == "END OF DATA":
                print("Fin de la réception des données.")
                break  # Exit the loop if "END OF DATA" is received
            file.write(line + "\n")  # Write the line to the file
            print(line)  # Optionally display the data in the console

ser.close()  # Close the serial port when done