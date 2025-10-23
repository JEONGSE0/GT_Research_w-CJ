# -*- coding: utf-8 -*-
import numpy as np
from scipy.signal import iirnotch, butter, lfilter_zi, lfilter

class RealtimeFilter:
    """
    8채널 EMG 실시간 필터링용
    - 60Hz, 120Hz 노치
    - 20~100Hz 밴드패스
    """
    def __init__(self, fs=250):
        self.fs = fs
        self.notch_freqs = [60, 120]
        self.b_bp, self.a_bp = butter(4, [20/(fs/2), 100/(fs/2)], btype='band')
        self.notches = [iirnotch(f0, Q=30, fs=fs) for f0 in self.notch_freqs]

        self.zi_bp = [lfilter_zi(self.b_bp, self.a_bp) for _ in range(8)]
        self.zi_notch = [[lfilter_zi(bn, an) for (bn, an) in self.notches] for _ in range(8)]

    def apply(self, sample):
        """입력: 8ch numpy array → 출력: 필터링된 8ch"""
        filtered = np.zeros(8, dtype=float)
        for ch in range(8):
            x = np.array([sample[ch]], dtype=float)
            for i, (bn, an) in enumerate(self.notches):
                x, self.zi_notch[ch][i] = lfilter(bn, an, x, zi=self.zi_notch[ch][i])
            x, self.zi_bp[ch] = lfilter(self.b_bp, self.a_bp, x, zi=self.zi_bp[ch])
            filtered[ch] = x[-1]
        return filtered
