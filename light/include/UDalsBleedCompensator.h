/*
 * Copyright (C) 2024-2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
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
    void setSampledRgb(float r, float g, float b, bool isLinear = false);
    bool compensate(float rawLux, float* outCompensatedLux);

#ifdef UDALS_TEST_ACCESSORS
    float calculateBleedLux() const {
        std::lock_guard<std::mutex> lock(mLock);
        return calculateBleedLuxLocked();
    }
    uint32_t getBacklight() const {
        std::lock_guard<std::mutex> lock(mLock);
        return mBacklight;
    }
    bool isDcMode() const {
        std::lock_guard<std::mutex> lock(mLock);
        return mDcMode;
    }
    bool isHbmMode() const {
        std::lock_guard<std::mutex> lock(mLock);
        return mHbmMode;
    }
    void getSampledRgb(float* r, float* g, float* b) const {
        std::lock_guard<std::mutex> lock(mLock);
        if (r) *r = mNormR;
        if (g) *g = mNormG;
        if (b) *b = mNormB;
    }
    float getLastReportedLux() const {
        std::lock_guard<std::mutex> lock(mLock);
        return mLastReportedLux;
    }
#endif

  private:
    float calculateBleedLuxLocked() const;

    static float interpolateTable(const std::array<CaliPoint, kTableSize>& table,
                                  float brightness);

    mutable std::mutex mLock;
    uint32_t mBacklight;
    bool mDcMode;
    bool mHbmMode;
    float mNormR;
    float mNormG;
    float mNormB;

    // 3-sample median ring buffer and hysteresis
    std::array<float, 3> mMedianWindow;
    size_t mMedianCount;
    size_t mMedianIndex;
    float mLastReportedLux;
    bool mHasReported;
};
