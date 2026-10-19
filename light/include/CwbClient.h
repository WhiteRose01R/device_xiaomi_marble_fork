/*
 * Copyright (C) 2024-2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

#include "UDalsBleedCompensator.h"

// Client for sampling display pixels under the ambient light sensor aperture.
class CwbClient {
  public:
    CwbClient(std::shared_ptr<UDalsBleedCompensator> compensator);
    ~CwbClient();

    bool start();
    void stop();
    bool requestCwbSample();

  private:
    void captureWorkerThread();

    std::shared_ptr<UDalsBleedCompensator> mCompensator;
    std::atomic<bool> mRunning;
    std::thread mWorker;

    std::mutex mLock;
    std::condition_variable mCv;
};
