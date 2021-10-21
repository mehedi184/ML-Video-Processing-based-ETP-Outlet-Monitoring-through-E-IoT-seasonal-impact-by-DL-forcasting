# -*- coding: utf-8 -*-
"""RGB_color_analysis.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1mQCESGyax6eBaWVB2qvNQgFpnXCCVmSb
"""

import numpy as np
import pandas as pd
import cv2
import matplotlib.pyplot as plt

"""### Read the data"""

from google.colab import drive
drive.mount('/content/drive')

df = pd.read_csv('/content/RGB.csv')
df2= pd.read_csv('/content/WQI_s.csv')
df

"""Convert frames to RGB"""

cap= cv2.VideoCapture('/content/drive/MyDrive/videoetp/etp.mp4')
i = 0
frame_skip = 60
while (cap.isOpened()):
  ret, frame = cap.read()
  if not ret:
    break
    
  if i > frame_skip - 1:
    b = frame[:, :, :1] 
    g = frame[:, :, 1:2] 
    r = frame[:, :, 2:] 
  
    # computing the mean 
    b_mean = np.mean(b) 
    g_mean = np.mean(g) 
    r_mean = np.mean(r)
    df=df.append({'Red': r_mean,'Green': g_mean,'Blue': b_mean},ignore_index=True)
    i = 0
    continue
  
  i += 1
cap.release()
cv2.destroyAllWindows()

df

df2

df2_mean = df2.mean()
df0=df2.replace(np.nan, df2_mean)
df0

print(df.dtypes)
print(df0.dtypes)

df1 = pd.DataFrame() 
df1=pd.merge(df0, df, left_index=True, right_index=True)
df1

"""Plotting"""

df1.plot(0,7, kind = 'line', ylim=(0,175), figsize=(15,8), color='orange', fontsize=14)
df1.plot(0,-1, kind = 'line', color='Blue', figsize=(15,8), fontsize=16)
df1.plot(0,-2, kind = 'line', color='green', figsize=(15,8), fontsize=16)
df1.plot(0,-3, kind = 'line', color='red', figsize=(15,8), fontsize=16)
df1.plot(0, [-1,7,-2,-3], kind = 'line', figsize=(15,8), fontsize=16)
ax = plt.gca()
ax.set_ylim([0,175])
plt.show()