/*
 * Copyright (C) 2024-2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>

// Compensator for under-display ambient light sensor optical screen bleed.
class UDalsBleedCompensator {
  public:
    // Calibration table row structure: {brightness, clear}
    struct CaliPoint {
        float brightness;
        float clear;
    };

    static constexpr size_t kTableSize = 24;

    UDalsBleedCompensator();
    ~UDalsBleedCompensator() = default;

    void reset();
    void setBacklight(uint32_t brightness);
    void setDcMode(bool dcMode);
    void setHbmMode(bool hbm);
    void setSampledRgb(float r, float g, float b);
    float compensate(float rawLux);

  private:
    float calculateBleedLuxLocked() const;

    static float interpolateTable(const std::array<CaliPoint, kTableSize>& table,
                                  float brightness);

    mutable std::mutex mLock;
    uint32_t mBacklight;
    bool mDcMode;
    bool mHbmMode;
    bool mHasSample;
    float mNormR;
    float mNormG;
    float mNormB;
    std::chrono::steady_clock::time_point mResetTime{};

    // 3-sample median ring buffer
    std::array<float, 3> mMedianWindow;
    size_t mMedianCount;
    size_t mMedianIndex;
};
