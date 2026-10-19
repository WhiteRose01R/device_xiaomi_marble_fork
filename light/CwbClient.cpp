/*
 * Copyright (C) 2024-2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "UDalsCwbClient"

#include "CwbClient.h"

#include <android-base/logging.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

static const char* kRgbDeviceNodes[] = {
    "/dev/udals_rgb",
    "/data/vendor/sensor/udals_rgb",
    nullptr
};

CwbClient::CwbClient(std::shared_ptr<UDalsBleedCompensator> compensator)
    : mCompensator(std::move(compensator)),
      mRunning(false) {}

CwbClient::~CwbClient() {
    stop();
}

bool CwbClient::start() {
    std::lock_guard<std::mutex> lock(mLock);
    if (mRunning) {
        return true;
    }

    mRunning = true;
    mWorker = std::thread(&CwbClient::captureWorkerThread, this);
    LOG(INFO) << "UDALS CWB capture worker started";
    return true;
}

void CwbClient::stop() {
    {
        std::lock_guard<std::mutex> lock(mLock);
        if (!mRunning) {
            return;
        }
        mRunning = false;
    }

    // Wake up worker immediately without waiting for sample interval sleep
    mCv.notify_all();

    if (mWorker.joinable()) {
        mWorker.join();
    }
    LOG(INFO) << "UDALS CWB capture worker stopped";
}

bool CwbClient::requestCwbSample() {
    if (!mRunning || mCompensator == nullptr) {
        return false;
    }

    // Check if hardware IPC device node or file is available
    for (int i = 0; kRgbDeviceNodes[i] != nullptr; ++i) {
        int fd = open(kRgbDeviceNodes[i], O_RDONLY | O_CLOEXEC);
        if (fd >= 0) {
            // Guard against stale regular files if external producer pauses or dies
            struct stat st{};
            if (fstat(fd, &st) == 0 && S_ISREG(st.st_mode)) {
                struct timespec now{};
                clock_gettime(CLOCK_REALTIME, &now);
                int64_t ageMs = (static_cast<int64_t>(now.tv_sec) - st.st_mtim.tv_sec) * 1000 +
                                (now.tv_nsec - st.st_mtim.tv_nsec) / 1000000;
                if (ageMs > 1000) {
                    // Ignore samples older than 1 second
                    close(fd);
                    continue;
                }
            }

            alignas(float) char buf[64] = {0};
            ssize_t n = read(fd, buf, sizeof(buf) - 1);
            close(fd);

            bool success = false;
            if (n > 0) {
                // Check for ASCII text first
                if (buf[0] == '.' || (buf[0] >= '0' && buf[0] <= '9') || buf[0] == ' ') {
                    float r = 0.0f, g = 0.0f, b = 0.0f;
                    if (sscanf(buf, "%f %f %f", &r, &g, &b) == 3) {
                        mCompensator->setSampledRgb(r, g, b, true /* already linear */);
                        success = true;
                    }
                }
                // Check for binary floats (3 floats = 12 bytes or UdalsSharedState >= 20 bytes)
                if (!success && (n == 12 || n >= 20)) {
                    float rgb[3] = {0.0f, 0.0f, 0.0f};
                    std::memcpy(rgb, buf, sizeof(rgb));
                    if (std::isfinite(rgb[0]) && std::isfinite(rgb[1]) && std::isfinite(rgb[2])) {
                        mCompensator->setSampledRgb(rgb[0], rgb[1], rgb[2], true /* already linear */);
                        success = true;
                    }
                }
            }
            std::memset(buf, 0, sizeof(buf));
            if (success) {
                return true;
            }
        }
    }

    // Fail-safe to zero bleed if no fresh/valid sample is available
    mCompensator->setSampledRgb(0.0f, 0.0f, 0.0f, true);
    return false;
}

void CwbClient::captureWorkerThread() {
    pthread_setname_np(pthread_self(), "UDalsCwbWorker");

    // 10 Hz sampling interval
    const auto kSampleInterval = std::chrono::milliseconds(100);

    while (mRunning) {
        requestCwbSample();

        std::unique_lock<std::mutex> lock(mLock);
        mCv.wait_for(lock, kSampleInterval, [this] { return !mRunning; });
    }
}
