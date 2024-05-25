#!/Library/Frameworks/Python.framework/Versions/3.9/bin/python3
import serial
import numpy as np
from tensorflow.keras.models import load_model
import matplotlib.pyplot as plt
from matplotlib import cm
import serial.tools.list_ports
import re

def read_serial_data():
    # Détection des ports série disponibles
    ports = serial.tools.list_ports.comports()
    for port, desc, hwid in sorted(ports):
        print("{}: {} [{}]".format(port, desc, hwid))

    print("Nom du port série: ", port)

    # Configure the serial port, replace '/dev/tty.usbmodem21201' with your Arduino's COM port
    ser = serial.Serial(port, 460800)

    # Initialiser la matrice pour les coefficients MFCC
    mfcc_matrix = []
    data_started = False

    # Lire les données série
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if line == "Sortie RESEAU":
                print("Fin de la réception des données.")
                break  # Sortir de la boucle si "Sortie RESEAU" est reçu
            if line.startswith("NUM DE FRAME"):
                data_started = True
                continue  # Ignorer les lignes de numéro de frame
            if data_started:
                # Ajouter les coefficients MFCC à la matrice
                mfcc_values = list(map(float, line.split()))
                mfcc_matrix.append(mfcc_values)
                print(line)  # Afficher les données dans la console pour vérification

    ser.close()  # Fermer le port série
    return mfcc_matrix

def predict_with_model(mfcc_matrix):
    # Vérifier la taille de la matrice et la convertir en tableau numpy
    if len(mfcc_matrix) == 48:
        mfcc_matrix = np.array(mfcc_matrix).reshape((1, 48, 13, 1))

        # Charger le modèle enregistré en utilisant le format natif Keras
        model = load_model('/Users/mathiasrechsteiner/Desktop/Cours ING 3/Semestre 2/Traitement du Signal/CNN_TensorFlow.keras', compile=False)

        # Utilisation de mfcc_matrix pour faire des prédictions
        predictions = model.predict(mfcc_matrix)
        print('\nPrédictions pour testFinal :')
        print(predictions)

        # Optionnel : Affichage de la classe prédite
        predicted_class = np.argmax(predictions, axis=-1)
        print('\nClasse prédite pour testFinal :')
        print(predicted_class)

        if predicted_class==0:
            print("ALERTE !")
        else:
            print("STOP !")

    else:
        print("Erreur : la matrice MFCC n'a pas la taille attendue.")

def graph_mfcc(mfcc_matrix):
    # Convertir en numpy array et transposer
    mfcc_matrix = np.array(mfcc_matrix).T

    # Plot MFCC with inverted axes
    plt.figure(figsize=(10, 4))
    plt.imshow(mfcc_matrix, cmap=cm.coolwarm, origin='lower', aspect='auto')
    plt.title('MFCC Graph over time')
    plt.xlabel('Time')
    plt.ylabel('MFCC Coefficients')
    plt.tight_layout()
    plt.show()

def main():
    mfcc_matrix = read_serial_data()
    predict_with_model(mfcc_matrix)
    graph_mfcc(mfcc_matrix)

if __name__ == "__main__":
    main()