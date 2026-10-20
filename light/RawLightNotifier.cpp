/*
 * Copyright (C) 2024-2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "RawLightNotifier"

#include "RawLightNotifier.h"

#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/parseint.h>
#include <android-base/strings.h>
#include <android-base/unique_fd.h>
#include <display/drm/mi_disp.h>
#include <poll.h>
#include <sys/ioctl.h>

#include "SensorNotifierUtils.h"
#include "SscCalApi.h"

static const std::string kDispFeatureDevice = "/dev/mi_display/disp_feature";

// sensor: xiaomi.sensor.ambientlight.raw
static const uint32_t kSensorTypeAmbientlightRaw = 33171111;

using android::hardware::Return;
using android::hardware::Void;
using android::hardware::sensors::V1_0::Event;

namespace {

class RawLightSensorCallback : public IEventQueueCallback {
  public:
    explicit RawLightSensorCallback(std::shared_ptr<UDalsBleedCompensator> compensator)
        : mCompensator(std::move(compensator)) {}

    Return<void> onEvent(const Event& e) override {
        float rawLux = e.u.vec4.y;
        float compensatedLux = rawLux;

        if (mCompensator != nullptr) {
            compensatedLux = mCompensator->compensate(rawLux);
        }

        _oem_msg msg{};
        msg.notifyType = REPORT_VALUE;
        msg.value = compensatedLux;
        msg.notifyTypeFloat = static_cast<float>(msg.notifyType);
        msg.unknown1 = 2.0f;
        msg.unknown2 = 5.0f;
        msg.sensorType = kSensorTypeAmbientlightRaw;

        SscCalApiWrapper::getInstance().processMsg(&msg);

        LOG(VERBOSE) << "UDALS event: raw=" << rawLux
                     << " compensated=" << compensatedLux
                     << " bleed=" << (rawLux - compensatedLux);

        return Void();
    }

  private:
    std::shared_ptr<UDalsBleedCompensator> mCompensator;
};

static uint32_t readPanelBacklight() {
    static const char* const kBacklightPaths[] = {
        "/sys/class/backlight/panel0-backlight/brightness",
        "/sys/class/backlight/panel1-backlight/brightness",
        nullptr
    };
    for (int i = 0; kBacklightPaths[i] != nullptr; ++i) {
        std::string content;
        if (android::base::ReadFileToString(kBacklightPaths[i], &content)) {
            uint32_t bkl = 0;
            if (android::base::ParseUint(android::base::Trim(content), &bkl)) {
                return bkl;
            }
        }
    }
    return 0;
}

}  // namespace

RawLightNotifier::RawLightNotifier(sp<ISensorManager> manager)
    : SensorNotifier(manager),
      isEnable(false),
      mCompensator(std::make_shared<UDalsBleedCompensator>()),
      mCwbClient(std::make_unique<CwbClient>(mCompensator)) {
    uint32_t initBkl = readPanelBacklight();
    if (initBkl > 0 && mCompensator != nullptr) {
        mCompensator->setBacklight(initBkl);
    }
    initializeSensorQueue("xiaomi.sensor.ambientlight.factory", false,
                          new RawLightSensorCallback(mCompensator));
    LOG(INFO) << "RawLightNotifier initialized with UDALS screen-bleed compensator (seed backlight="
              << initBkl << ")";
}

RawLightNotifier::~RawLightNotifier() {
    deactivate();
    if (mCwbClient != nullptr) {
        mCwbClient->stop();
    }
    if (isEnable && mQueue != nullptr) {
        mQueue->disableSensor(mSensorHandle);
        isEnable = false;
    }
}

void RawLightNotifier::notify() {
    Result res;

    android::base::unique_fd disp_fd_ =
            android::base::unique_fd(open(kDispFeatureDevice.c_str(), O_RDWR));
    if (disp_fd_.get() == -1) {
        LOG(ERROR) << "failed to open " << kDispFeatureDevice;
    }

    // Enable the sensor initially
    res = mQueue->enableSensor(mSensorHandle, 20000 /* sample period */, 0 /* latency */);
    if (res != Result::OK) {
        LOG(ERROR) << "failed to enable sensor";
    } else {
        isEnable = true;
        if (mCwbClient != nullptr) {
            mCwbClient->start();
        }
    }

    // Register for power events
    const std::vector<disp_event_type> notifyEvents = {MI_DISP_EVENT_POWER, MI_DISP_EVENT_FPS,
                                                       MI_DISP_EVENT_51_BRIGHTNESS,
                                                       MI_DISP_EVENT_HBM, MI_DISP_EVENT_DC};

    for (const disp_event_type& event : notifyEvents) {
        disp_event_req req;
        req.base.flag = 0;
        req.base.disp_id = MI_DISP_PRIMARY;
        req.type = event;
        ioctl(disp_fd_.get(), MI_DISP_IOCTL_REGISTER_EVENT, &req);
    }

    struct pollfd dispEventPoll = {
            .fd = disp_fd_.get(),
            .events = POLLIN,
    };

    _oem_msg msg{};
    notify_t notifyType;
    float value;

    while (mActive) {
        int rc = poll(&dispEventPoll, 1, -1);
        if (rc < 0) {
            LOG(ERROR) << "failed to poll " << kDispFeatureDevice << ", err: " << rc;
            continue;
        }

        std::shared_ptr<disp_event_resp> response = parseDispEvent(disp_fd_.get());
        if (response == nullptr) {
            continue;
        }

        if (response->base.type == MI_DISP_EVENT_POWER) {
            notifyType = POWER_STATE;
            value = response->data[0];
            switch (response->data[0]) {
                case MI_DISP_POWER_ON:
                    if (mCompensator != nullptr) {
                        mCompensator->reset();
                        uint32_t bkl = readPanelBacklight();
                        if (bkl > 0) {
                            mCompensator->setBacklight(bkl);
                        }
                    }
                    if (!isEnable) {
                        res = mQueue->enableSensor(mSensorHandle, 20000 /* sample period */,
                                                   0 /* latency */);
                        if (res != Result::OK) {
                            LOG(ERROR) << "failed to enable sensor";
                        } else {
                            isEnable = true;
                            if (mCwbClient != nullptr) {
                                mCwbClient->start();
                            }
                        }
                    }
                    break;
                default:
                    if (isEnable) {
                        res = mQueue->disableSensor(mSensorHandle);
                        if (res != Result::OK) {
                            LOG(ERROR) << "failed to disable sensor";
                        } else {
                            isEnable = false;
                        }
                        if (mCwbClient != nullptr) {
                            mCwbClient->stop();
                        }
                        if (mCompensator != nullptr) {
                            mCompensator->reset();
                        }
                    }
                    break;
            }
        } else {
            switch (response->base.type) {
                case MI_DISP_EVENT_FPS:
                    notifyType = DISPLAY_FREQUENCY;
                    value = response->data[0];
                    break;
                case MI_DISP_EVENT_51_BRIGHTNESS:
                    notifyType = BRIGHTNESS;
                    value = *(uint16_t*)response->data;
                    if (mCompensator != nullptr) {
                        mCompensator->setBacklight(static_cast<uint32_t>(value));
                    }
                    break;
                case MI_DISP_EVENT_HBM:
                    notifyType = BRIGHTNESS;
                    value = response->data[0] ? -1 : -2;
                    if (mCompensator != nullptr) {
                        mCompensator->setHbmMode(response->data[0] != 0);
                    }
                    break;
                case MI_DISP_EVENT_DC:
                    notifyType = DC_STATE;
                    value = response->data[0];
                    if (mCompensator != nullptr) {
                        mCompensator->setDcMode(response->data[0] != 0);
                    }
                    break;
                default:
                    LOG(ERROR) << "got unknown event: " << response->base.type;
                    continue;
            }
        }
        msg = {};
        msg.sensorType = kSensorTypeAmbientlightRaw;
        msg.notifyType = notifyType;
        msg.notifyTypeFloat = static_cast<float>(notifyType);
        msg.value = value;
        msg.unknown1 = 1.0f;
        msg.unknown2 = 5.0f;

        SscCalApiWrapper::getInstance().processMsg(&msg);
    }
}
