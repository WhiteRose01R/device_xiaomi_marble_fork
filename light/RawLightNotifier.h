/*
 * Copyright (C) 2024-2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <memory>

#include "CwbClient.h"
#include "SensorNotifier.h"
#include "UDalsBleedCompensator.h"

// Notifier for under-display ambient light sensor with optical screen bleed compensation.
class RawLightNotifier : public SensorNotifier {
  public:
    RawLightNotifier(sp<ISensorManager> manager);
    ~RawLightNotifier() override;

  protected:
    void notify() override;

  private:
    bool isEnable;
    std::shared_ptr<UDalsBleedCompensator> mCompensator;
    std::unique_ptr<CwbClient> mCwbClient;
};
