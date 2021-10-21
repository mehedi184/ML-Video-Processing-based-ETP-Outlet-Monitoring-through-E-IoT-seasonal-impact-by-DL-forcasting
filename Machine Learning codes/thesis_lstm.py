# -*- coding: utf-8 -*-
"""Thesis LSTM

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1hmKDMxaoXAjuEclDXqLZADQD-USS7fn1

#Data Pre processed 
fire the system
"""

# Commented out IPython magic to ensure Python compatibility.
# Import modules and packages
import numpy as np
import pandas as pd
import matplotlib.font_manager
import matplotlib.pyplot as plt
import datetime as dt
from datetime import datetime
from sklearn.metrics import mean_squared_error
from math import sqrt
from keras import metrics
from keras.callbacks import EarlyStopping, ReduceLROnPlateau, ModelCheckpoint, TensorBoard
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import MinMaxScaler
# %matplotlib inline

# Commented out IPython magic to ensure Python compatibility.
# %pwd

!git clone https://github.com/mehedi184/-Machine-Learning-based-Real-Time-ETP-Outlet-Monitoring-through-E-IoT.git

"""Read data"""

df = pd.read_csv('/content/WQI_s.csv')
df

df.plot('Date', ['Temperature', 'pH','DO','Turbidity(V)','TDS','EC', 'WQI'],figsize=(18,6),ylim=(0,66000))

df_mean = df.mean()
dataset_train=df.replace(np.nan, df_mean)
dataset_train

dataset_train.plot(figsize=(14,8))
plt.show()

dataset_train.plot(0, [1,2,3,4,5,6,7,8,9,10], subplots=True, figsize=(15,12))

print(dataset_train.dtypes)

"""#Data processing

Removing all commas and convert data to matrix shape format.


"""

# Select features (columns) to be involved intro training and predictions
cols = list(dataset_train)[1:8]

# Extract dates (will be used in visualization)
datelist_train = list(dataset_train['Date'])
datelist_train = [dt.datetime.strptime(date, '%m/%d/%Y %H:%M:%S') for date in datelist_train]

print('Training set shape == {}'.format(dataset_train.shape))
print('All timestamps == {}'.format(len(datelist_train)))
print('Featured selected: {}'.format(cols))

datelist_train

dataset_train = pd.DataFrame(dataset_train, columns=cols)
dataset_train.index = datelist_train
dataset_train.index = pd.to_datetime(dataset_train.index)

dataset_train = dataset_train[cols].astype(str)
for i in cols:
    for j in range(0, len(dataset_train)):
        dataset_train[i][j] = dataset_train[i][j].replace(',', '')
dataset_train = dataset_train.astype(float)

# Using multiple features (predictors)
training_set = dataset_train.to_numpy()

print('Shape of training set == {}.'.format(training_set.shape))
training_set

#installing mglearn -- just takes 5s to install -- it's a built in function to understand the difference between the different scalers.
!pip install mglearn

import mglearn
#this is just a built in thing to understand the difference between the different scalers
mglearn.plots.plot_scaling()

# Feature Scaling
from sklearn.preprocessing import StandardScaler

sc = StandardScaler()
training_set_scaled = sc.fit_transform(training_set)

sc_predict = StandardScaler()
sc_predict.fit_transform(training_set[:, 6:7])

# Creating a data structure with 90 timestamps and 1 output
X_train = []
y_train = []

n_future = 3  # Number of days we want top predict into the future
n_past = 5    # Number of past days we want to use to predict the future

for i in range(n_past, len(training_set_scaled) - n_future +1):
    X_train.append(training_set_scaled[i - n_past:i, 0:dataset_train.shape[1] - 1])
    y_train.append(training_set_scaled[i + n_future - 1:i + n_future, 6:7])

X_train, y_train = np.array(X_train), np.array(y_train)

print('X_train shape == {}.'.format(X_train.shape))
print('y_train shape == {}.'.format(y_train.shape))

X_train=X_train.astype(int)
y_train=y_train.astype(int)

"""#Create a model Training

Building the LSTM based Neural Network
"""

# Import Libraries and packages from Keras
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import LSTM
from keras.layers import Dropout
from keras.optimizers import Adam
from sklearn.metrics import accuracy_score
from keras.optimizers import SGD
import tensorflow as tf

# Initializing the Neural Network based on LSTM
model = Sequential()
 
# Adding 1st LSTM layer
model.add(LSTM(units=64, return_sequences=True, input_shape=(n_past, dataset_train.shape[1]-1)))
model.add(Dropout(0.5)) 
# Adding 2nd LSTM layer
model.add(LSTM(units=32, return_sequences=False))

# Adding Dropout
model.add(Dense(units=25,activation='relu'))
model.add(Dropout(0.2))
model.add(Dense(units=10,activation='sigmoid'))
model.add(Dropout(0.2))
model.add(Dense(units=1, activation='linear'))
# Compiling the Neural Network
model.compile(optimizer = Adam(learning_rate=0.01), loss='mean_squared_error',metrics=['accuracy'])
model.summary()

"""Start training"""

# Commented out IPython magic to ensure Python compatibility.
# %%time
# es = EarlyStopping(monitor='val_loss', min_delta=1e-10, patience=10, verbose=1)
# rlr = ReduceLROnPlateau(monitor='val_loss', factor=0.5, patience=10, verbose=1)
# mcp = ModelCheckpoint(filepath='weights.h5', monitor='val_loss', verbose=1, save_best_only=True, save_weights_only=True)
# 
# tb = TensorBoard('logs')
# 
# history = model.fit(X_train, y_train,
#                     shuffle=True,
#                     epochs=100,
#                     callbacks=[es, rlr, mcp, tb],
#                     validation_split=0.2 ,
#                     validation_data=(X_train, y_train),
#                     verbose=1,
#                     batch_size=64)

def plot_graphs(history, string):
  plt.title(['Training and validation '+string])
  plt.plot(history.history[string])
  plt.plot(history.history['val_'+string])
  plt.xlabel("Epochs")
  plt.ylabel(string)
  plt.legend([string, 'val_'+string])
  plt.show()
  
plot_graphs(history, "loss")
plot_graphs(history, "accuracy")
print(history.history['loss'])
print(history.history['val_loss'])

print(history.history['accuracy'])
print(history.history['val_accuracy'])

"""#Make future predictions"""

# Generate list of sequence of days for predictions
datelist_future = pd.date_range(datelist_train[-1], periods=n_future, freq='1d').tolist()

'''
Remeber, we have datelist_train from begining.
'''

# Convert Pandas Timestamp to Datetime object (for transformation) --> FUTURE
datelist_future_ = []
for this_timestamp in datelist_future:
    datelist_future_.append(this_timestamp.date().ctime())

datelist_future

# Perform predictions
predictions_future = model.predict(X_train[-n_future:])

predictions_train = model.predict(X_train[n_past:])

predictions_future.shape

predictions_train.shape

# Inverse the predictions to original measurements

# ---> Special function: convert <datetime.date> to <Timestamp>
def datetime_to_timestamp(x):
    '''
        x : a given datetime value (datetime.datetime)
    '''
    return datetime.strptime(x.strftime('%m/%d/%Y %H:%M:%S'), '%m/%d/%Y %H:%M:%S')


y_pred_future = sc_predict.inverse_transform(predictions_future)
y_pred_train = sc_predict.inverse_transform(predictions_train)

""" Make predictions for future dates"""

PREDICTIONS_FUTURE = pd.DataFrame(y_pred_future, columns=['WQI']).set_index(pd.Series(datelist_future))
PREDICTION_TRAIN = pd.DataFrame(y_pred_train, columns=['WQI']).set_index(pd.Series(datelist_train[2 * n_past + n_future -1:]))

# Convert <datetime.date> to <Timestamp> for PREDCITION_TRAIN
PREDICTION_TRAIN.index = PREDICTION_TRAIN.index.to_series().apply(datetime_to_timestamp)

PREDICTION_TRAIN=round(PREDICTION_TRAIN)
PREDICTION_TRAIN

PREDICTION_TRAIN.shape

PREDICTIONS_FUTURE=round(PREDICTIONS_FUTURE)

PREDICTIONS_FUTURE

PREDICTIONS_FUTURE.shape

plt.ion()

"""Visualize the Predictions"""

# Set plot size 
from pylab import rcParams
rcParams['figure.figsize'] = 14, 5

# Plot parameters
START_DATE_FOR_PLOTTING = '8/15/2020 00:00:00'

plt.plot(PREDICTIONS_FUTURE.index, PREDICTIONS_FUTURE['WQI'], color='r', label='Forcasted WQI')
plt.plot(PREDICTION_TRAIN.loc[START_DATE_FOR_PLOTTING:].index, PREDICTION_TRAIN.loc[START_DATE_FOR_PLOTTING:]['WQI'], color='orange', label='Training predictions')
plt.plot(dataset_train.loc[START_DATE_FOR_PLOTTING:].index, dataset_train.loc[START_DATE_FOR_PLOTTING:]['WQI'], color='b', label='Actual WQI')

plt.axvline(x = min(PREDICTIONS_FUTURE.index), color='green', linewidth=2, linestyle='--')

plt.grid(which='major', color='#cccccc', alpha=0.5)

plt.legend(shadow=True)
plt.title('Predcitions , Forecasting and Acutal WQI', family='Arial', fontsize=12)
plt.xlabel('Date', family='Arial', fontsize=10)
plt.ylabel('WQI ', family='Arial', fontsize=10)
plt.xticks(rotation=45, fontsize=8)
ax = plt.gca()
ax.set_ylim([0,110])
plt.show()

"""Evaluating Time Forcasting's prediction"""

total_rmse = 0
for i in range(len(PREDICTION_TRAIN)-3):

        rmse = np.sqrt(mean_squared_error(PREDICTION_TRAIN[i:i+3], PREDICTIONS_FUTURE))
        rmse=rmse*6
        print('t+%d RMSE: %f' % ((i+5), rmse))
        total_rmse += rmse

print('total rmse: ', total_rmse)
print('actual acc:',(total_rmse/(len(PREDICTION_TRAIN))) ,'%')

# Using sklearn
from sklearn.metrics import r2_score
print(r2_score(PREDICTION_TRAIN[i:i+3], PREDICTIONS_FUTURE))

"""#Walk Forward Validation

Using keras models with scikit-learn pipelines
"""

series= PREDICTION_TRAIN
series

# split data into train and test
X = series.values   # X is still a column vector with 36 rows
print(X)

series1=PREDICTIONS_FUTURE

Y= series1.values

#X = dataset
train, test = X[:-1502], Y[-1502:]
train

test

# walk-forward validation
history = [x for x in train]
predictions = list()
for i in range(len(test)):
	# make prediction
	predictions.append(history[-1])
	# observation
	history.append(test[i])

    # report performance
rmse = sqrt(mean_squared_error(test, predictions))
#rmse=rmse/3
print('RMSE: %.3f' % (rmse))