# perceptron_pacman.py
# --------------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).

import util
from pacman import GameState
import random
import numpy as np
from pacman import Directions
import math
import numpy as np
from featureExtractors import FEATURE_NAMES

PRINT = True


class PerceptronPacman:

    def __init__(self, num_train_iterations=20, learning_rate=1):

        self.max_iterations = num_train_iterations
        self.learning_rate = learning_rate

        # A list of which features to include by name. To exclude a feature comment out the line with that feature name
        feature_names_to_use = [
            'closestFood', 
            # 'closestFoodNow',
            'closestGhost',
            'closestGhostNow',
            'closestScaredGhost',
            'closestScaredGhostNow',
            'eatenByGhost',
            'eatsCapsule',
            # 'eatsFood',
            # "foodCount",
            # 'foodWithinFiveSpaces',
            # 'foodWithinNineSpaces',
            # 'foodWithinThreeSpaces',  
            # 'furthestFood', 
            'numberAvailableActions',
            "ratioCapsuleDistance",
            # "ratioFoodDistance",
            "ratioGhostDistance",
            "ratioScaredGhostDistance"
            ]
        
        # we start our indexing from 1 because the bias term is at index 0 in the data set
        feature_name_to_idx = dict(zip(FEATURE_NAMES, np.arange(1, len(FEATURE_NAMES) + 1)))

        # a list of the indices for the features that should be used. We always include 0 for the bias term.
        self.features_to_use = [0] + [feature_name_to_idx[feature_name] for feature_name in feature_names_to_use]

        "*** YOUR CODE HERE ***"
        #Xavier Initialization
        limit = np.sqrt(6 / (len(self.features_to_use) + 1))
        self.weights = np.random.uniform(-limit, limit, len(self.features_to_use))

    def predict(self, feature_vector):
        """
        This function should take a feature vector as a numpy array and pass it through your perceptron and output activation function

        THE FEATURE VECTOR WILL HAVE AN ENTRY FOR BIAS ALREADY AT INDEX 0.
        """
        # filter the data to only include your chosen features. We might not need to do this if we're working with training data that has already been filtered.
        if len(feature_vector) > len(self.features_to_use):
            vector_to_classify = feature_vector[self.features_to_use]
        else:
            vector_to_classify = feature_vector

        #Pass feature vector through perceptron
        x = np.matmul(self.weights, vector_to_classify)  
        return self.activationOutput(x) 


    def activationHidden(self, x):
        """
        Implement your chosen activation function for any hidden layers here.
        """
        #Sigmoid Activation function to np array
        try:
            return np.array([1/(1 + math.exp(-x[i])) for i in range(len(x))])
        except:
            return 1/(1 + math.exp(-x))

    def activationOutput(self, x):
        """
        Implement your chosen activation function for the output here.
        """
        #Sigmoid Activation function to np array
        try:
            return np.array([1/(1 + math.exp(-x[i])) for i in range(len(x))])
        except:
            return 1/(1 + math.exp(-x))
        
    def evaluate(self, data, labels):
        """
        This function should take a data set and corresponding labels and compute the performance of the perceptron.
        You might for example use accuracy for classification, but you can implement whatever performance measure
        you think is suitable. You aren't evaluated what you choose here. 
        This function is just used for you to assess the performance of your training.

        The data should be a 2D numpy array where each row is a feature vector

        THE FEATURE VECTOR WILL HAVE AN ENTRY FOR BIAS ALREADY AT INDEX 0.

        The labels should be a list of 1s and 0s, where the value at index i is the
        corresponding label for the feature vector at index i in the appropriate data set. For example, labels[1]
        is the label for the feature at data[1]
        """

        # filter the data to only include your chosen features
        X_eval = data[:, self.features_to_use]

        return np.mean((self.activationOutput(np.matmul(X_eval,self.weights)) - labels)**2)


    def train(self, trainingData, trainingLabels, validationData, validationLabels):
        import matplotlib.pyplot as plt
        """
        This function should take training and validation data sets and train the perceptron

        The training and validation data sets should be 2D numpy arrays where each row is a different feature vector

        THE FEATURE VECTOR WILL HAVE AN ENTRY FOR BIAS ALREADY AT INDEX 0.

        The training and validation labels should be a list of 1s and 0s, where the value at index i is the
        corresponding label for the feature vector at index i in the appropriate data set. For example, trainingLabels[1]
        is the label for the feature at trainingData[1]
        """

        # filter the data to only include your chosen features. Use the validation data however you like.
        X_train = trainingData[:, self.features_to_use]
        X_validate = validationData[:, self.features_to_use]

        val_acc = []
        train_loss = []
        for _ in range(self.max_iterations):
            # forward pass 
            y = self.activationOutput(np.matmul(X_train,self.weights))
            train_loss.append(self.evaluate(trainingData, trainingLabels))
            # update weights
            dW = (2/len(trainingData)) * np.dot(X_train.T, (y - trainingLabels))
            self.weights -= self.learning_rate * dW
            val_acc.append(self.evaluate(validationData, validationLabels))
        
        #Plotting the training loss and validation loss
        plt.plot(range(self.max_iterations), train_loss, label='Training Loss')
        plt.plot(range(self.max_iterations), val_acc, label='Validation Loss')
        plt.xlabel('Iterations')
        plt.ylabel('Loss')
        plt.title('Loss vs Iterations')
        plt.legend()
        plt.grid()
        plt.show()


    def save_weights(self, weights_path):
        """
        Saves your weights to a .model file. You're free to format this however you like.
        For example with a single layer perceptron you could just save a single line with all the weights.
        """
       
        np.savetxt(weights_path, self.weights, delimiter=",")

    def load_weights(self, weights_path):
        """
        Loads your weights from a .model file. 
        Whatever you do here should work with the formatting of your save_weights function.
        """

        self.weights = np.loadtxt(weights_path, delimiter=",")
