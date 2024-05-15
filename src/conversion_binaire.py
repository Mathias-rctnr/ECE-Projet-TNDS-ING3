import struct

def convert_to_binary(input_path, output_path):
    # Lire les valeurs à partir du fichier texte
    with open(input_path, 'r') as file:
        lines = file.readlines()

    values = []
    for line in lines:
        try:
            # Essayer de convertir chaque ligne en entier
            number = int(line.strip())
            values.append(number)
        except ValueError:
            # Si la conversion échoue, sauter cette ligne
            continue

    # Ouvrir le fichier binaire en mode écriture
    with open(output_path, 'wb') as f:
        for value in values:
            # Convertir chaque valeur entière en une représentation binaire
            binary = struct.pack('<h', value)
            # Écrire la représentation binaire dans le fichier
            f.write(binary)


convert_to_binary("/Users/mathiasrechsteiner/Desktop/Cours ING 3/Semestre 2/Traitement du Signal/output.txt", "/Users/mathiasrechsteiner/Desktop/Cours ING 3/Semestre 2/Traitement du Signal/output_binary.bin")
convert_to_binary("/Users/mathiasrechsteiner/Desktop/Cours ING 3/Semestre 2/Traitement du Signal/NonFiltre.txt", "/Users/mathiasrechsteiner/Desktop/Cours ING 3/Semestre 2/Traitement du Signal/NonFiltre_binary.bin")
convert_to_binary("/Users/mathiasrechsteiner/Desktop/Cours ING 3/Semestre 2/Traitement du Signal/Filtre.txt", "/Users/mathiasrechsteiner/Desktop/Cours ING 3/Semestre 2/Traitement du Signal/Filtre_binary.bin")
