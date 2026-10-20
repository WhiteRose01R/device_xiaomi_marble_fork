/*
 * Copyright (C) 2024-2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

#include "UDalsBleedCompensator.h"

struct AHardwareBuffer;
struct native_handle;

namespace DisplayConfig {
class ClientInterface;
}
class CwbCallbackHandler;

// Client for sampling display pixels under the ambient light sensor aperture via Qualcomm CWB.
class CwbClient {
  public:
    explicit CwbClient(std::shared_ptr<UDalsBleedCompensator> compensator);
    ~CwbClient();

    bool start();
    void stop();

  private:
    void captureWorkerThread();
    bool ensureCwbInitialized();
    void releaseCwb();
    bool requestCwbSample();

    std::shared_ptr<UDalsBleedCompensator> mCompensator;
    std::atomic<bool> mRunning;
    std::thread mWorker;

    std::mutex mLock;
    std::condition_variable mCv;

    void* mDpyConfigLib = nullptr;
    DisplayConfig::ClientInterface* mCwbClient = nullptr;
    CwbCallbackHandler* mCallbackHandler = nullptr;
    void (*mDestroyFn)(DisplayConfig::ClientInterface*) = nullptr;
    AHardwareBuffer* mCwbBuffer = nullptr;
    const struct native_handle* mCwbBufferHandle = nullptr;
    uint32_t mBufferStride = 0;
    uint32_t mInitFailures = 0;
    uint32_t mPendingTimeouts = 0;
    std::chrono::steady_clock::time_point mLastInitAttempt{};
};
