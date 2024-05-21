#!/Library/Frameworks/Python.framework/Versions/3.9/bin/python3
import serial
print("PySerial version:", serial.VERSION)

import serial.tools.list_ports

import re
import numpy as np

import matplotlib.pyplot as plt
from matplotlib import cm

def graphMFCC():
    # Define the desired MFCC size
    n_mfcc = 13  # Number of MFCC coefficients

    # Load MFCC coefficients from text file
    mfcc_path = '/Users/mathiasrechsteiner/Desktop/Cours ING 3/Semestre 2/Traitement du Signal/MFCC.txt'  # Correct the path to your actual data file
    mfcc_coeffs = np.loadtxt(mfcc_path)

    # Reshape the coefficients to the desired size
    n_frames = mfcc_coeffs.shape[0] // n_mfcc
    mfcc_coeffs = mfcc_coeffs[:n_mfcc * n_frames].reshape(n_frames, n_mfcc)

    # Transpose the array to invert the axes
    mfcc_coeffs = mfcc_coeffs.T

    # Plot MFCC with inverted axes
    plt.figure(figsize=(10, 4))
    plt.imshow(mfcc_coeffs, cmap=cm.coolwarm, origin='lower', aspect='auto')
    plt.title('MFCC Graph over time')
    plt.xlabel('Time')
    plt.ylabel('MFCC Coefficients')
    plt.tight_layout()
    plt.show()

def main():
    ports = serial.tools.list_ports.comports()
    for port, desc, hwid in sorted(ports):
        print("{}: {} [{}]".format(port, desc, hwid))

    print("Nom du port série: ", port)

    # Configure the serial port, replace 'COMx' with your Arduino's COM port
    ser = serial.Serial(port, 460800)

    # Chemin du fichier où écrire les données
    output_file_path = '/Users/mathiasrechsteiner/Desktop/Cours ING 3/Semestre 2/Traitement du Signal/MFCC.txt'

    # Ouvrir le fichier pour l'écriture
    with open(output_file_path, 'w') as file:
        try:
            while True:
                # Lire une ligne du port série
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                # Utiliser une expression régulière pour extraire les valeurs MFCC
                if 'MFCC[' in line:
                    mfcc_value = re.findall(r'MFCC\[\d+\]: ([\d\.]+)', line)
                    if mfcc_value:
                        # Écrire la valeur MFCC extraite dans le fichier
                        file.write(mfcc_value[0] + '\n')
                        file.flush()  # Assure that each value is written immediately to the file

        except KeyboardInterrupt:
            # Fermer le port série lorsqu'on interrompt le script manuellement
            ser.close()
            print("Arrêt du script.")
            graphMFCC()  # Call the graphing function after data collection is stopped

if __name__ == "__main__":
    main()