# -*- coding: utf-8 -*-
import torch
import torch.nn as nn
import torch.nn.functional as F

class EMG_CNN(nn.Module):
    def __init__(self, num_classes=8):
        super().__init__()
        self.conv1 = nn.Conv1d(8, 32, kernel_size=5, stride=1, padding=2)
        self.bn1 = nn.BatchNorm1d(32)
        self.conv2 = nn.Conv1d(32, 64, kernel_size=5, stride=1, padding=2)
        self.bn2 = nn.BatchNorm1d(64)
        self.conv3 = nn.Conv1d(64, 128, kernel_size=3, stride=1, padding=1)
        self.bn3 = nn.BatchNorm1d(128)
        self.pool = nn.MaxPool1d(2)

        # 윈도우 길이와 무관하게 동작
        self.gap = nn.AdaptiveAvgPool1d(1)
        self.fc1 = nn.Linear(128, 128)
        self.fc2 = nn.Linear(128, num_classes)

    def forward(self, x):      # x: (N, 8, W)
        x = self.pool(F.relu(self.bn1(self.conv1(x))))  # /2
        x = self.pool(F.relu(self.bn2(self.conv2(x))))  # /4
        x = self.pool(F.relu(self.bn3(self.conv3(x))))  # /8
        x = self.gap(x).squeeze(-1)                     # (N,128)
        x = F.relu(self.fc1(x))
        return self.fc2(x)
