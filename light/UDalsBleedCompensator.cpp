/*
 * Copyright (C) 2024-2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "UDalsBleedCompensator"

#include "UDalsBleedCompensator.h"

#include <android-base/logging.h>
#include <algorithm>
#include <cmath>

namespace {

// Factory calibration table for normal PWM mode (from lightSensorConfig.json panel_Info_cali_ori)
// Rows: {brightness, clear}
constexpr std::array<UDalsBleedCompensator::CaliPoint, 24> kNormalDataR = {{
    {0.0f, 0.0f},
    {10.0f, 40.001358f},
    {50.0f, 105.003563f},
    {100.0f, 185.006287f},
    {150.0f, 265.009003f},
    {200.0f, 345.011719f},
    {250.0f, 425.014435f},
    {300.0f, 510.017303f},
    {350.0f, 590.02002f},
    {400.0f, 645.021912f},
    {450.0f, 700.023743f},
    {500.0f, 765.026001f},
    {550.0f, 830.028198f},
    {600.0f, 900.030579f},
    {650.0f, 980.033264f},
    {800.0f, 1235.04187f},
    {950.0f, 1395.047363f},
    {1100.0f, 1560.052979f},
    {1250.0f, 1725.058594f},
    {1400.0f, 1900.064453f},
    {1550.0f, 2100.071289f},
    {1700.0f, 2305.078369f},
    {1850.0f, 2540.086182f},
    {2047.0f, 2855.096924f},
}};

constexpr std::array<UDalsBleedCompensator::CaliPoint, 24> kNormalDataG = {{
    {0.0f, 0.0f},
    {10.0f, 30.001019f},
    {50.0f, 110.003738f},
    {100.0f, 220.007477f},
    {150.0f, 315.010681f},
    {200.0f, 420.014252f},
    {250.0f, 515.017456f},
    {300.0f, 620.021057f},
    {350.0f, 700.023743f},
    {400.0f, 770.026123f},
    {450.0f, 850.02887f},
    {500.0f, 935.031738f},
    {550.0f, 1025.03479f},
    {600.0f, 1125.038208f},
    {650.0f, 1230.041748f},
    {800.0f, 1590.053955f},
    {950.0f, 1825.062012f},
    {1100.0f, 2030.06897f},
    {1250.0f, 2270.077148f},
    {1400.0f, 2525.085693f},
    {1550.0f, 2800.094971f},
    {1700.0f, 3115.105713f},
    {1850.0f, 3415.115967f},
    {2047.0f, 3895.132324f},
}};

constexpr std::array<UDalsBleedCompensator::CaliPoint, 24> kNormalDataB = {{
    {0.0f, 0.0f},
    {10.0f, 10.00034f},
    {50.0f, 35.00119f},
    {100.0f, 55.001869f},
    {150.0f, 80.002716f},
    {200.0f, 105.003563f},
    {250.0f, 130.00441f},
    {300.0f, 155.005264f},
    {350.0f, 180.006119f},
    {400.0f, 200.00679f},
    {450.0f, 220.007477f},
    {500.0f, 240.008148f},
    {550.0f, 265.009003f},
    {600.0f, 290.009857f},
    {650.0f, 320.010864f},
    {800.0f, 410.013916f},
    {950.0f, 470.015961f},
    {1100.0f, 520.017639f},
    {1250.0f, 585.019836f},
    {1400.0f, 655.022217f},
    {1550.0f, 730.02478f},
    {1700.0f, 810.027527f},
    {1850.0f, 900.030579f},
    {2047.0f, 1030.034912f},
}};

// Factory calibration table for DC Dimming mode (from lightSensorConfig.json panel_Info_cali_ori_dc)
constexpr std::array<UDalsBleedCompensator::CaliPoint, 24> kDcDataR = {{
    {0.0f, 0.0f},
    {10.0f, 81.0f},
    {50.0f, 279.5f},
    {100.0f, 537.0f},
    {150.0f, 776.5f},
    {200.0f, 1036.5f},
    {250.0f, 1287.5f},
    {300.0f, 1551.5f},
    {350.0f, 1813.0f},
    {400.0f, 2092.0f},
    {450.0f, 2406.5f},
    {500.0f, 2663.0f},
    {550.0f, 2933.5f},
    {600.0f, 3198.0f},
    {650.0f, 3467.5f},
    {800.0f, 4236.0f},
    {950.0f, 5128.5f},
    {1100.0f, 5998.0f},
    {1250.0f, 6874.0f},
    {1400.0f, 7743.5f},
    {1550.0f, 8575.5f},
    {1700.0f, 9616.5f},
    {1850.0f, 10535.5f},
    {2047.0f, 11636.5f},
}};

constexpr std::array<UDalsBleedCompensator::CaliPoint, 24> kDcDataG = {{
    {0.0f, 0.0f},
    {10.0f, 131.5f},
    {50.0f, 403.5f},
    {100.0f, 733.5f},
    {150.0f, 1071.0f},
    {200.0f, 1398.0f},
    {250.0f, 1727.5f},
    {300.0f, 2072.0f},
    {350.0f, 2410.5f},
    {400.0f, 2768.5f},
    {450.0f, 3179.0f},
    {500.0f, 3499.0f},
    {550.0f, 3820.5f},
    {600.0f, 4140.5f},
    {650.0f, 4463.5f},
    {800.0f, 5451.5f},
    {950.0f, 6460.0f},
    {1100.0f, 7485.0f},
    {1250.0f, 8549.0f},
    {1400.0f, 9611.5f},
    {1550.0f, 10640.0f},
    {1700.0f, 11710.0f},
    {1850.0f, 12781.0f},
    {2047.0f, 14236.0f},
}};

constexpr std::array<UDalsBleedCompensator::CaliPoint, 24> kDcDataB = {{
    {0.0f, 0.0f},
    {10.0f, 50.0f},
    {50.0f, 147.5f},
    {100.0f, 267.5f},
    {150.0f, 383.5f},
    {200.0f, 507.5f},
    {250.0f, 621.0f},
    {300.0f, 742.5f},
    {350.0f, 849.0f},
    {400.0f, 975.5f},
    {450.0f, 1112.0f},
    {500.0f, 1226.5f},
    {550.0f, 1347.0f},
    {600.0f, 1462.5f},
    {650.0f, 1580.0f},
    {800.0f, 1905.5f},
    {950.0f, 2250.0f},
    {1100.0f, 2614.5f},
    {1250.0f, 2963.5f},
    {1400.0f, 3294.5f},
    {1550.0f, 3634.5f},
    {1700.0f, 3996.5f},
    {1850.0f, 4335.5f},
    {2047.0f, 4794.0f},
}};

// Peak optical bleed (in lux) at max brightness (2047) derived empirically on POCO F5:
// Pure Red: 63.50 lux, Pure Green: 237.89 lux, Pure Blue: 44.82 lux
// Pure White: 333.29 lux (Sum: 63.50 + 237.89 + 44.82 = 346.21 ~= 333.29, linearity error < 3.8%)
constexpr float kPeakBleedLuxRed = 63.50f;
constexpr float kPeakBleedLuxGreen = 237.89f;
constexpr float kPeakBleedLuxBlue = 44.82f;

inline float medianOfThree(float a, float b, float c) {
    return std::max(std::min(a, b), std::min(std::max(a, b), c));
}

}  // namespace

UDalsBleedCompensator::UDalsBleedCompensator()
    : mBacklight(0),
      mDcMode(false),
      mHbmMode(false),
      mHasSample(false),
      mNormR(0.0f),
      mNormG(0.0f),
      mNormB(0.0f),
      mResetTime(std::chrono::steady_clock::now()),
      mMedianWindow{},
      mMedianCount(0),
      mMedianIndex(0) {}

void UDalsBleedCompensator::reset() {
    std::lock_guard<std::mutex> lock(mLock);
    mMedianWindow.fill(0.0f);
    mMedianCount = 0;
    mMedianIndex = 0;
    mBacklight = 0;
    mHbmMode = false;
    mHasSample = false;
    mNormR = 0.0f;
    mNormG = 0.0f;
    mNormB = 0.0f;
    mResetTime = std::chrono::steady_clock::now();
}

void UDalsBleedCompensator::setBacklight(uint32_t brightness) {
    std::lock_guard<std::mutex> lock(mLock);
    mBacklight = std::min(brightness, 2047u);
}

void UDalsBleedCompensator::setDcMode(bool dcMode) {
    std::lock_guard<std::mutex> lock(mLock);
    mDcMode = dcMode;
}

void UDalsBleedCompensator::setHbmMode(bool hbm) {
    std::lock_guard<std::mutex> lock(mLock);
    mHbmMode = hbm;
}

void UDalsBleedCompensator::setSampledRgb(float r, float g, float b) {
    std::lock_guard<std::mutex> lock(mLock);
    mNormR = std::clamp(r, 0.0f, 1.0f);
    mNormG = std::clamp(g, 0.0f, 1.0f);
    mNormB = std::clamp(b, 0.0f, 1.0f);
    mHasSample = true;
}

float UDalsBleedCompensator::interpolateTable(
        const std::array<CaliPoint, kTableSize>& table, float brightness) {
    if (brightness <= table.front().brightness) {
        return 0.0f;
    }
    if (brightness >= table.back().brightness) {
        return table.back().clear;
    }

    for (size_t i = 0; i < table.size() - 1; ++i) {
        if (brightness >= table[i].brightness && brightness <= table[i + 1].brightness) {
            float b0 = table[i].brightness;
            float b1 = table[i + 1].brightness;
            float t = (brightness - b0) / (b1 - b0);
            return table[i].clear + t * (table[i + 1].clear - table[i].clear);
        }
    }
    return 0.0f;
}

float UDalsBleedCompensator::calculateBleedLuxLocked() const {
    uint32_t effectiveBacklight = mHbmMode ? 2047u : mBacklight;

    if (effectiveBacklight == 0 || (mNormR <= 0.001f && mNormG <= 0.001f && mNormB <= 0.001f)) {
        return 0.0f;
    }

    const auto& tableR = mDcMode ? kDcDataR : kNormalDataR;
    const auto& tableG = mDcMode ? kDcDataG : kNormalDataG;
    const auto& tableB = mDcMode ? kDcDataB : kNormalDataB;

    float maxClearR = tableR.back().clear;
    float maxClearG = tableG.back().clear;
    float maxClearB = tableB.back().clear;

    float clearAtBrightnessR = interpolateTable(tableR, static_cast<float>(effectiveBacklight));
    float clearAtBrightnessG = interpolateTable(tableG, static_cast<float>(effectiveBacklight));
    float clearAtBrightnessB = interpolateTable(tableB, static_cast<float>(effectiveBacklight));

    float curveR = (maxClearR > 0.0f) ? (clearAtBrightnessR / maxClearR) : 0.0f;
    float curveG = (maxClearG > 0.0f) ? (clearAtBrightnessG / maxClearG) : 0.0f;
    float curveB = (maxClearB > 0.0f) ? (clearAtBrightnessB / maxClearB) : 0.0f;

    // Subpixel bleed sum
    float bleedR = mNormR * curveR * kPeakBleedLuxRed;
    float bleedG = mNormG * curveG * kPeakBleedLuxGreen;
    float bleedB = mNormB * curveB * kPeakBleedLuxBlue;

    return std::max(0.0f, bleedR + bleedG + bleedB);
}

float UDalsBleedCompensator::compensate(float rawLux) {
    std::lock_guard<std::mutex> lock(mLock);

    // Suppress uncompensated flare spikes in the brief (~30-50ms) window after wake
    // before the first CWB capture frame is ready, falling back after 150ms.
    if (!mHasSample && mBacklight > 0) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - mResetTime).count();
        if (elapsed < 150) {
            return 0.0f;
        }
    }

    float bleed = calculateBleedLuxLocked();
    float compensated = std::max(0.0f, rawLux - bleed);

    mMedianWindow[mMedianIndex] = compensated;
    mMedianIndex = (mMedianIndex + 1) % mMedianWindow.size();
    if (mMedianCount < mMedianWindow.size()) {
        mMedianCount++;
    }

    if (mMedianCount == 1) {
        return mMedianWindow[0];
    }
    if (mMedianCount == 2) {
        return std::min(mMedianWindow[0], mMedianWindow[1]);
    }
    return medianOfThree(mMedianWindow[0], mMedianWindow[1], mMedianWindow[2]);
}
