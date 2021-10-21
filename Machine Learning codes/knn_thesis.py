# -*- coding: utf-8 -*-
"""knn_thesis.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1GHwaFg9fGDyUFPQ4akxT0GrswJJ-d3ri
"""

# Commented out IPython magic to ensure Python compatibility.
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np
# %matplotlib inline
from sklearn.preprocessing import StandardScaler
import seaborn as sns
import warnings
warnings.filterwarnings('ignore')

df1 = pd.read_csv("/content/ETP.csv",index_col=0)
df1

df2=df1.drop(['WQI','WQC','Verdict'], axis = 1) 
df2

df_mean = df2.mean()
df=df2.replace(np.nan, df_mean)
df

df.describe()

df.info()

df3=df1.drop(['WQI','WQC','Numeric_WQC'], axis = 1) 
df3

df_mean = df3.mean()
df4=df3.replace(np.nan, df_mean)
df4

fig, ax = plt.subplots(ncols=4, nrows=2, figsize=(20,10))
index = 0
ax = ax.flatten()

for col, value in df.items():
    if col != 'Date':
        sns.boxplot(y=col, data=df, ax=ax[index])
        index += 1
plt.tight_layout(pad=0.5, w_pad=0.7, h_pad=5.0)

plt.figure(figsize=(11,7))
sns.countplot(df['Numeric_WQC'])

plt.figure(figsize=(11,7))
sns.countplot(df4['Verdict'])

corr = df.corr()
plt.figure(figsize=(20,10))
sns.heatmap(corr, annot=True, cmap='coolwarm')

sns.FacetGrid(df,hue=None,size=5).map(sns.distplot,'Numeric_WQC').add_legend()

scaler = StandardScaler()

scaler.fit(df.drop('Numeric_WQC',axis=1))

scaled_features = scaler.transform(df.drop('Numeric_WQC',axis=1))

df_feat = pd.DataFrame(scaled_features,columns=df.columns[:-1])
df_feat.head()

from sklearn.model_selection import train_test_split

from sklearn.neighbors import KNeighborsClassifier
from sklearn.metrics import classification_report,confusion_matrix

X_train, X_test, y_train, y_test = train_test_split(scaled_features,df['Numeric_WQC'],
                                                    test_size=0.30)

error_rate = []

for i in range(1,30):
    
    knn = KNeighborsClassifier(n_neighbors=i)
    knn.fit(X_train,y_train)
    pred_i = knn.predict(X_test)
    error_rate.append(np.mean(pred_i != y_test))

plt.figure(figsize=(10,6))
plt.plot(range(1,30),error_rate,color='blue', linestyle='dashed', marker='o',
         markerfacecolor='blue', markersize=10)
plt.title('Error Rate vs. K Value')
plt.xlabel('K')
plt.ylabel('Error Rate')

knn = KNeighborsClassifier(n_neighbors=5)
knn.fit(X_train,y_train)
pred = knn.predict(X_test)

print(confusion_matrix(y_test,pred))
print(classification_report(y_test,pred))

from sklearn.metrics import roc_curve
from sklearn.metrics import auc

scaler.fit(df4.drop('Verdict',axis=1))
df_feat = pd.DataFrame(scaled_features,columns=df.columns[:-1])
scaled_features = scaler.transform(df4.drop('Verdict',axis=1))
X_train, X_test, y_train, y_test = train_test_split(scaled_features,df4['Verdict'],
                                                    test_size=0.30)
knn = KNeighborsClassifier(n_neighbors=5)
knn.fit(X_train,y_train)
pred = knn.predict(X_test)
print(confusion_matrix(y_test,pred))
print(classification_report(y_test,pred))

fpr, tpr, threshold = roc_curve(y_test, pred)
roc_auc = auc(fpr, tpr)
plt.figure(figsize=(10,6))
plt.plot(fpr, tpr, 'b', label = 'AUC = %0.2f' % roc_auc)
plt.legend(loc = 'lower right')
plt.plot([0, 1], [0, 1],'r--')
plt.xlim([0, 1])
plt.ylim([0, 1])
plt.ylabel('True Positive Rate')
plt.xlabel('False Positive Rate')
plt.title('ROC Curve of kNN')
plt.show()

X = df4.drop(columns=['Verdict'])
y = df4['Verdict']

y.value_counts()

from imblearn.over_sampling import SMOTE
oversample = SMOTE(k_neighbors=7)
# transform the dataset
X, y = oversample.fit_resample(X, y)

from sklearn.model_selection import cross_val_score
def classify(model, X, y):
    x_train, x_test, y_train, y_test = train_test_split(X, y, test_size=0.25, random_state=64)
    # train the model
    model.fit(x_train, y_train)
    print("Accuracy:", model.score(x_test, y_test) * 100)
    
    # cross-validation
    score = cross_val_score(model, X, y, cv=3)
    print("CV Score:", np.mean(score)*100)

from sklearn.linear_model import LogisticRegression
model = LogisticRegression()
classify(model, X, y)

from sklearn.tree import DecisionTreeClassifier
model = DecisionTreeClassifier()
classify(model, X, y)

from sklearn.ensemble import RandomForestClassifier
model = RandomForestClassifier()
classify(model, X, y)

import xgboost as xgb
model = xgb.XGBClassifier()
classify(model, X, y)

# pred_prob = knn.predict_proba(X_test)

# fpr = {}
# tpr = {}
# thresh ={}

# n_class = 4

# for i in range(n_class):    
#     fpr[i], tpr[i], thresh[i] = roc_curve(y_test, pred_prob[:,i], pos_label=i)
    
# # plotting    
# plt.plot(fpr[0], tpr[0],color='orange', label='Class 1')
# plt.plot(fpr[1], tpr[1],color='green', label='Class 2')
# plt.plot(fpr[2], tpr[2],color='blue', label='Class 3')
# plt.plot(fpr[3], tpr[3],color='black', label='Class 4')

# plt.title('Multiclass ROC curve')
# plt.xlabel('False Positive Rate')
# plt.ylabel('True Positive rate')
# plt.legend(loc='best')