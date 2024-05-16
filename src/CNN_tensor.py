import numpy as np
import tensorflow as tf
from tensorflow.keras import layers, models
import time
import DATA

# Les données d'entraînement et de test (remplacer les ellipses avec vos données réelles)
training_data = DATA.training
training_labels = np.array([0] * 50 + [1] * 50)
test_data = DATA.test
test_labels = np.array([0] * 5 + [1] * 5)

# Convertir les données en tableaux NumPy
training_data = np.array(training_data)
test_data = np.array(test_data)

# Redimensionnement des données d'entraînement et de test
training_data = training_data.reshape((100, 48, 13, 1))
test_data = test_data.reshape((10, 48, 13, 1))

# Définition du modèle CNN
model = models.Sequential()
model.add(layers.Input(shape=(48, 13, 1)))
model.add(layers.Conv2D(32, (3, 3), activation='relu'))
model.add(layers.MaxPooling2D((2, 2)))
model.add(layers.Conv2D(64, (3, 3), activation='relu'))
model.add(layers.MaxPooling2D((2, 2)))
model.add(layers.Flatten())
model.add(layers.Dense(64, activation='relu'))
model.add(layers.Dense(2, activation='softmax'))

# Compilation du modèle
model.compile(optimizer='adam',
              loss='sparse_categorical_crossentropy',
              metrics=['accuracy'])

# Entraînement du modèle
start_time = time.time()
history = model.fit(training_data, training_labels, epochs=200, batch_size=5, validation_data=(test_data, test_labels))
training_duration = time.time() - start_time

# Évaluation du modèle
test_loss, test_acc = model.evaluate(test_data, test_labels, verbose=2)

# Calcul de la MSE sur les données d'entraînement et de test (pour l'affichage seulement)
train_predictions = model.predict(training_data)
test_predictions = model.predict(test_data)

train_mse = np.mean((training_labels - np.argmax(train_predictions, axis=1))**2)
test_mse = np.mean((test_labels - np.argmax(test_predictions, axis=1))**2)

# Récapitulatif des informations importantes en français
print('\nRésumé de l\'entraînement :')
print(f'Précision finale sur les données de test : {test_acc:.4f}')
print(f'Perte finale sur les données de test : {test_loss:.4f}')
print(f'MSE finale sur les données de test : {test_mse:.4f}')
print(f'Durée totale de l\'entraînement : {training_duration:.2f} secondes')
print(f'MSE finale sur les données d\'entraînement : {train_mse:.4f}')
print(f'Nombre total de cycles (epochs) : {len(history.history["accuracy"])}')

# Historique de l'entraînement
#print('\nHistorique de l\'entraînement :')
#print('Époque\tPrécision Entraînement\tPerte Entraînement\tPrécision Validation\tPerte Validation')
#for epoch in range(len(history.history['accuracy'])):
    #print(f'{epoch + 1}\t{history.history["accuracy"][epoch]:.4f}\t\t{history.history["loss"][epoch]:.4f}\t\t{history.history["val_accuracy"][epoch]:.4f}\t\t{history.history["val_loss"][epoch]:.4f}')

# Vérification de la condition de validation
if test_mse < 0.05:
    print("Le modèle satisfait la condition de validation : MSE sur les données de test est inférieure à 0.05")
else:
    print("Le modèle ne satisfait pas la condition de validation : MSE sur les données de test est supérieure ou égale à 0.05")