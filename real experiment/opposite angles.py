#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr 25 17:18:10 2025

@author: mohsen
"""

import pandas as pd
import numpy as np

df = pd.read_csv('timestamp_bee_phi.txt')

df['timestamp'] = df['timestamp'] - df['timestamp'].iloc[0]

df.to_csv('your_file_shifted.txt', index=False)

print(df)

df = pd.read_csv('your_file_shifted.txt')

df['opposite_phi'] = df['bee_phi'] + 180

df[['timestamp', 'opposite_phi']].to_csv('your_file_opposite.txt', index=False)

print(df[['timestamp', 'opposite_phi']])