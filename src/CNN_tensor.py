import numpy as np
import tensorflow as tf
from tensorflow.keras import layers, models
import time
import DATA

# Définition de l'utilisation des biais
IS_BIASED = True

# Activation de la précision en 32 bits pour les nombres flottants
tf.keras.backend.set_floatx('float32')

# Importation des données d'entraînement et de test
training_data = DATA.training
training_labels = np.array([0] * 50 + [1] * 50)
test_data = DATA.test
test_labels = np.array([0] * 5 + [1] * 5)

# Assurez-vous de redimensionner testFinal correctement
testFinal = DATA.dataTest

# Conversion et redimensionnement des données
training_data = np.array(training_data).reshape((100, 48, 13, 1))
test_data = np.array(test_data).reshape((10, 48, 13, 1))
testFinal = np.array(testFinal).reshape((1, 48, 13, 1))  # Assurez-vous que testFinal a la bonne forme

# Définition et compilation du modèle CNN
model = models.Sequential([
    layers.Input(shape=(48, 13, 1)),
    layers.Conv2D(8, (3, 3), activation='relu'),  # Réduit à 8 filtres
    layers.MaxPooling2D((2, 2)),
    layers.Flatten(),
    layers.Dense(16, activation='relu'),  # Réduit à 16 unités
    layers.Dense(2, activation='softmax')
])
model.compile(optimizer='adam', loss='sparse_categorical_crossentropy', metrics=['accuracy'])

# Entraînement et évaluation du modèle
start_time = time.time()
history = model.fit(training_data, training_labels, epochs=100, batch_size=5, validation_data=(test_data, test_labels))
training_duration = time.time() - start_time
test_loss, test_acc = model.evaluate(test_data, test_labels, verbose=2)

# Affichage des résultats
print('\nRésumé de l\'entraînement :')
print(f'Précision finale sur les données de test : {test_acc:.4f}')
print(f'Perte finale sur les données de test : {test_loss:.4f}')
print(f'Durée totale de l\'entraînement : {training_duration:.2f} secondes')

# Rassembler tous les poids et biais dans des tableaux distincts
all_weights = []
all_biases = []
for i, layer in enumerate(model.layers):
    if len(layer.get_weights()) > 0:
        weights, biases = layer.get_weights()
        all_weights.append(weights)
        all_biases.append(biases)

# Fonction pour formater les poids et les biais en chaînes de caractères pour Arduino
def format_for_arduino(data, name):
    data_str = ", ".join(map(str, data))
    return f"const float {name}[] = {{{data_str}}};"

# Affichage des poids et des biais sous forme de tableaux distincts pour Arduino
#print('\n// All weights')
#print(format_for_arduino(all_weights, "model_weights"))
#print('\n// All biases')
#print(format_for_arduino(all_biases, "model_biases"))

print()
weights_biases = model.get_weights()

if IS_BIASED:
    print("#define _2_OPTIMIZE B00100000 // MULTIPLE_BIASES_PER_LAYER \n")
    print('float biases[] = {')
    for b in all_biases:
        print('  ', end='')
        for value in b:
            print(value, end=', ')
        print()
    print('};\n')
else:
    print("#define _2_OPTIMIZE B01000000 // NO_BIAS \n")

print('float weights[] = {', end="")
for w in all_weights:
    print()
    for row in w:
        print('  ', end='')
        for value in row.flatten():
            print(value, end=', ')
        print()
print('};\n')

# Utilisation de testFinal pour faire des prédictions
predictions = model.predict(testFinal)
print('\nPrédictions pour testFinal :')
print(predictions)

# Optionnel : Affichage de la classe prédite
predicted_class = np.argmax(predictions, axis=-1)
print('\nClasse prédite pour testFinal :')
print(predicted_class)