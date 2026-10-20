/*
 * Copyright (C) 2024-2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "UDalsCwbClient"

#include "CwbClient.h"

#include <android-base/logging.h>
#include <android/hardware_buffer.h>
#include <vndk/hardware_buffer.h>
#include <dlfcn.h>
#include <pthread.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

// TCS3701 under-display sensor aperture on marble: center (745, 51), radius 47
constexpr int32_t kRoiLeft = 698;
constexpr int32_t kRoiTop = 4;
constexpr int32_t kRoiRight = 792;
constexpr int32_t kRoiBottom = 98;
constexpr int32_t kRoiWidth = kRoiRight - kRoiLeft;   // 94
constexpr int32_t kRoiHeight = kRoiBottom - kRoiTop;  // 94

constexpr int32_t kCenterRelX = 47;
constexpr int32_t kCenterRelY = 47;
constexpr int32_t kRadius1 = 18;
constexpr int32_t kRadius2 = 34;
constexpr int32_t kRadius3 = 47;

// Qualcomm HWC primary display identifier
constexpr uint32_t kPrimaryDisplayId = 1;

std::array<float, 256> initGammaLut() {
    std::array<float, 256> lut{};
    for (size_t i = 0; i < 256; ++i) {
        lut[i] = std::pow(static_cast<float>(i) / 255.0f, 2.2f);
    }
    return lut;
}

const std::array<float, 256> kGammaLut = initGammaLut();

}  // namespace

namespace DisplayConfig {

enum class DisplayType {
    kInvalid = -1,
    kPrimary = 0,
    kExternal = 1,
    kVirtual = 2,
    kBuiltIn2 = 3,
};

enum class ExternalStatus {
    kOffline = 0,
    kOnline = 1,
    kPause = 2,
    kResume = 3,
};

enum class DynRefreshRateOp {
    kDisableMetadata = 0,
    kEnableMetadata = 1,
    kSetFps = 2,
};

struct Rect {
    int left = 0;
    int top = 0;
    int right = 0;
    int bottom = 0;
};

class ConfigCallback {
public:
    virtual void NotifyCWBBufferDone(int32_t error, const native_handle_t* buffer) = 0;
    virtual void NotifyQsyncChange(bool, int32_t, int32_t) {}
    virtual void NotifyIdleStatus(bool) {}
    virtual ~ConfigCallback() = default;
};

class ClientInterface {
public:
    virtual int IsDisplayConnected(DisplayType type, bool* connected) = 0;
    virtual int SetDisplayStatus(DisplayType type, ExternalStatus status) = 0;
    virtual int ConfigureDynRefreshRate(DynRefreshRateOp op, uint32_t hz) = 0;
    virtual int GetConfigCount(DisplayType type, uint32_t* count) = 0;
    virtual int GetActiveConfig(DisplayType type, uint32_t* index) = 0;
    virtual int SetActiveConfig(DisplayType type, uint32_t index) = 0;
    virtual int GetDisplayAttributes(uint32_t index, DisplayType type, void* attr) = 0;
    virtual int SetPanelBrightness(uint32_t level) = 0;
    virtual int GetPanelBrightness(uint32_t* level) = 0;
    virtual int MinHdcpEncryptionLevelChanged(DisplayType type, uint32_t min_enc_level) = 0;
    virtual int RefreshScreen() = 0;
    virtual int ControlPartialUpdate(DisplayType type, bool enable) = 0;
    virtual int ToggleScreenUpdate(bool on) = 0;
    virtual int SetIdleTimeout(uint32_t value) = 0;
    virtual int GetHDRCapabilities(DisplayType type, void* caps) = 0;
    virtual int SetCameraLaunchStatus(uint32_t on) = 0;
    virtual ~ClientInterface() = default;
    virtual int DisplayBWTransactionPending(bool* status) = 0;
    virtual int SetDisplayAnimating(uint64_t display_id, bool animating) = 0;
    virtual int ControlIdlePowerCollapse(bool enable, bool synchronous) = 0;
    virtual int GetWriteBackCapabilities(bool* status) = 0;
    virtual int SetDisplayDppsAdROI(uint32_t display_id, uint32_t h_start, uint32_t h_end,
                                   uint32_t v_start, uint32_t v_end, uint32_t factor_in,
                                   uint32_t factor_out) = 0;
    virtual int UpdateVSyncSourceOnPowerModeOff() = 0;
    virtual int UpdateVSyncSourceOnPowerModeDoze() = 0;
    virtual int SetPowerMode(uint32_t display_id, uint32_t power_mode) = 0;
    virtual int IsPowerModeOverrideSupported(uint32_t display_id, bool* supported) = 0;
    virtual int IsHDRSupported(uint32_t display_id, bool* supported) = 0;
    virtual int IsWCGSupported(uint32_t display_id, bool* supported) = 0;
    virtual int SetLayerAsMask(uint32_t display_id, uint64_t layer_id) = 0;
    virtual int GetDebugProperty(std::string prop_name, std::string* value) = 0;
    virtual int GetActiveBuiltinDisplayAttributes(void* attr) = 0;
    virtual int SetPanelLuminanceAttributes(uint32_t display_id, float min_lum, float max_lum) = 0;
    virtual int IsBuiltInDisplay(uint32_t display_id, bool* is_builtin) = 0;
    virtual int IsAsyncVDSCreationSupported(bool* supported) = 0;
    virtual int CreateVirtualDisplay(uint32_t width, uint32_t height, int format) = 0;
    virtual int GetSupportedDSIBitClks(uint32_t display_id, std::vector<uint64_t>* bit_clks) = 0;
    virtual int GetDSIClk(uint32_t display_id, uint64_t* bit_clk) = 0;
    virtual int SetDSIClk(uint32_t display_id, uint64_t bit_clk) = 0;
    virtual int SetCWBOutputBuffer(uint32_t display_id, Rect rect, bool dither,
                                   const native_handle_t* buffer) = 0;
};

}  // namespace DisplayConfig

class CwbCallbackHandler : public DisplayConfig::ConfigCallback {
public:
    void NotifyCWBBufferDone(int32_t error, const native_handle_t* /* buffer */) override {
        std::lock_guard<std::mutex> lock(mMutex);
        mError = error;
        mDone = true;
        mPending = false;
        mCv.notify_one();
    }

    void prepareForCapture() {
        std::lock_guard<std::mutex> lock(mMutex);
        mDone = false;
        mError = 0;
        mPending = true;
    }

    void cancelPending() {
        std::lock_guard<std::mutex> lock(mMutex);
        mPending = false;
    }

    bool isPending() const {
        std::lock_guard<std::mutex> lock(mMutex);
        return mPending;
    }

    bool consumeIfDone(int32_t* outError) {
        std::lock_guard<std::mutex> lock(mMutex);
        if (!mDone) {
            return false;
        }
        *outError = mError;
        mDone = false;
        return true;
    }

    bool waitForDone(std::chrono::milliseconds timeout, int32_t* outError) {
        std::unique_lock<std::mutex> lock(mMutex);
        if (!mCv.wait_for(lock, timeout, [this] { return mDone; })) {
            return false;
        }
        *outError = mError;
        mDone = false;
        return true;
    }

    void reset() {
        std::lock_guard<std::mutex> lock(mMutex);
        mDone = false;
        mPending = false;
        mError = 0;
    }

private:
    mutable std::mutex mMutex;
    std::condition_variable mCv;
    bool mDone = false;
    bool mPending = false;
    int32_t mError = 0;
};

CwbClient::CwbClient(std::shared_ptr<UDalsBleedCompensator> compensator)
    : mCompensator(std::move(compensator)),
      mRunning(false) {}

CwbClient::~CwbClient() {
    stop();
    releaseCwb();
}

bool CwbClient::ensureCwbInitialized() {
    if (mCwbClient != nullptr && mCwbBufferHandle != nullptr) {
        return true;
    }

    auto now = std::chrono::steady_clock::now();
    auto minInterval = (mInitFailures >= 15) ? std::chrono::seconds(5) : std::chrono::seconds(1);
    if (mInitFailures > 0 &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - mLastInitAttempt) < minInterval) {
        return false;
    }
    mLastInitAttempt = now;

    // Dynamic dlopen of libdisplayconfig.qti.so avoids hard build-time vendor coupling
    // and linker namespace conflicts across disparate QTI display HAL revisions.
    mDpyConfigLib = dlopen("libdisplayconfig.qti.so", RTLD_NOW);
    if (mDpyConfigLib == nullptr) {
        mInitFailures++;
        LOG(WARNING) << "Failed to dlopen libdisplayconfig.qti.so: " << dlerror()
                     << " (attempt " << mInitFailures << ")";
        releaseCwb();
        return false;
    }

    using CreateFn = int (*)(std::string, DisplayConfig::ConfigCallback*,
                             DisplayConfig::ClientInterface**);
    using DestroyFn = void (*)(DisplayConfig::ClientInterface*);

    auto createFn = reinterpret_cast<CreateFn>(
        dlsym(mDpyConfigLib,
              "_ZN13DisplayConfig15ClientInterface6CreateENSt3__112basic_stringIcNS1_11char_traitsIcEENS1_9allocatorIcEEEEPNS_14ConfigCallbackEPPS0_"));
    auto destroyFn = reinterpret_cast<DestroyFn>(
        dlsym(mDpyConfigLib, "_ZN13DisplayConfig15ClientInterface7DestroyEPS0_"));

    if (createFn == nullptr || destroyFn == nullptr) {
        mInitFailures++;
        LOG(WARNING) << "Failed to find DisplayConfig ClientInterface symbols"
                     << " (attempt " << mInitFailures << ")";
        releaseCwb();
        return false;
    }
    mDestroyFn = destroyFn;
    mCallbackHandler = new CwbCallbackHandler();

    DisplayConfig::ClientInterface* client = nullptr;
    if (createFn("sensor-notifier", mCallbackHandler, &client) != 0 || client == nullptr) {
        mInitFailures++;
        LOG(WARNING) << "DisplayConfig ClientInterface Create failed"
                     << " (attempt " << mInitFailures << ")";
        releaseCwb();
        return false;
    }
    mCwbClient = client;

    AHardwareBuffer_Desc desc = {
        .width = static_cast<uint32_t>(kRoiWidth),
        .height = static_cast<uint32_t>(kRoiHeight),
        .layers = 1,
        .format = AHARDWAREBUFFER_FORMAT_R8G8B8A8_UNORM,
        .usage = AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN |
                 AHARDWAREBUFFER_USAGE_GPU_COLOR_OUTPUT,
        .stride = 0,
        .rfu0 = 0,
        .rfu1 = 0,
    };

    if (AHardwareBuffer_allocate(&desc, &mCwbBuffer) != 0 || mCwbBuffer == nullptr) {
        mInitFailures++;
        LOG(WARNING) << "Failed to allocate 94x94 CWB buffer"
                     << " (attempt " << mInitFailures << ")";
        releaseCwb();
        return false;
    }

    AHardwareBuffer_describe(mCwbBuffer, &desc);
    mCwbBufferHandle = AHardwareBuffer_getNativeHandle(mCwbBuffer);
    mBufferStride = desc.stride;
    mInitFailures = 0;
    LOG(INFO) << "CwbClient initialized hardware CWB: 94x94 RGBA, stride=" << mBufferStride;
    return true;
}

void CwbClient::releaseCwb() {
    if (mCwbBuffer != nullptr) {
        AHardwareBuffer_release(mCwbBuffer);
        mCwbBuffer = nullptr;
    }
    mCwbBufferHandle = nullptr;

    if (mCwbClient != nullptr && mDestroyFn != nullptr) {
        mDestroyFn(mCwbClient);
        mCwbClient = nullptr;
    }

    if (mCallbackHandler != nullptr) {
        delete mCallbackHandler;
        mCallbackHandler = nullptr;
    }

    if (mDpyConfigLib != nullptr) {
        dlclose(mDpyConfigLib);
        mDpyConfigLib = nullptr;
    }
    mDestroyFn = nullptr;
}

bool CwbClient::start() {
    std::lock_guard<std::mutex> lock(mLock);
    if (mRunning) {
        return true;
    }

    if (mCallbackHandler != nullptr) {
        mCallbackHandler->reset();
    }
    mInitFailures = 0;
    mPendingTimeouts = 0;
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

    mCv.notify_all();

    if (mWorker.joinable()) {
        mWorker.join();
    }

    if (mCallbackHandler != nullptr) {
        mCallbackHandler->reset();
    }
    mPendingTimeouts = 0;

    if (mCompensator != nullptr) {
        mCompensator->setSampledRgb(0.0f, 0.0f, 0.0f);
    }
    LOG(INFO) << "UDALS CWB capture worker stopped";
}

bool CwbClient::requestCwbSample() {
    if (!mRunning || mCompensator == nullptr) {
        return false;
    }

    if (!ensureCwbInitialized()) {
        mCompensator->setSampledRgb(0.0f, 0.0f, 0.0f);
        return false;
    }

    int32_t cwbError = 0;
    bool bufferReady = mCallbackHandler->consumeIfDone(&cwbError);

    if (!bufferReady && mCallbackHandler->isPending()) {
        if (mCallbackHandler->waitForDone(std::chrono::milliseconds(20), &cwbError)) {
            bufferReady = true;
        } else if (++mPendingTimeouts < 5) {
            return false;
        } else {
            // HWC dropped the pending CWB request (>500ms); reset state
            mCallbackHandler->reset();
            mPendingTimeouts = 0;
        }
    }

    if (!bufferReady) {
        mPendingTimeouts = 0;
        DisplayConfig::Rect roiRect = {
            .left = kRoiLeft,
            .top = kRoiTop,
            .right = kRoiRight,
            .bottom = kRoiBottom,
        };

        mCallbackHandler->prepareForCapture();

        int ret = mCwbClient->SetCWBOutputBuffer(kPrimaryDisplayId, roiRect, true, mCwbBufferHandle);
        if (ret != 0) {
            mCallbackHandler->cancelPending();
            return false;
        }

        if (mCallbackHandler->waitForDone(std::chrono::milliseconds(80), &cwbError)) {
            bufferReady = true;
        } else {
            return false;
        }
    }

    if (cwbError != 0) {
        return false;
    }
    mPendingTimeouts = 0;

    void* vaddr = nullptr;
    if (AHardwareBuffer_lock(mCwbBuffer, AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN, -1, nullptr,
                             &vaddr) != 0 ||
        vaddr == nullptr) {
        return false;
    }

    const uint8_t* pixels = static_cast<const uint8_t*>(vaddr);
    float weightedR = 0.0f;
    float weightedG = 0.0f;
    float weightedB = 0.0f;
    float totalWeight = 0.0f;

    for (int32_t y = 0; y < kRoiHeight; ++y) {
        int32_t dy = y - kCenterRelY;
        int32_t dy2 = dy * dy;
        const uint8_t* row = pixels + (y * mBufferStride * 4);

        for (int32_t x = 0; x < kRoiWidth; ++x) {
            int32_t dx = x - kCenterRelX;
            int32_t distSq = dx * dx + dy2;

            if (distSq > kRadius3 * kRadius3) {
                continue;
            }

            float w;
            if (distSq <= kRadius1 * kRadius1) {
                w = 1.0f;
            } else if (distSq <= kRadius2 * kRadius2) {
                w = 0.50f;
            } else {
                w = 0.20f;
            }

            int32_t offset = x * 4;
            uint8_t r = row[offset];
            uint8_t g = row[offset + 1];
            uint8_t b = row[offset + 2];

            weightedR += kGammaLut[r] * w;
            weightedG += kGammaLut[g] * w;
            weightedB += kGammaLut[b] * w;
            totalWeight += w;
        }
    }

    AHardwareBuffer_unlock(mCwbBuffer, nullptr);

    if (totalWeight > 0.0f) {
        float normR = std::clamp(weightedR / totalWeight, 0.0f, 1.0f);
        float normG = std::clamp(weightedG / totalWeight, 0.0f, 1.0f);
        float normB = std::clamp(weightedB / totalWeight, 0.0f, 1.0f);
        mCompensator->setSampledRgb(normR, normG, normB);
        return true;
    }

    return false;
}

void CwbClient::captureWorkerThread() {
    pthread_setname_np(pthread_self(), "UDalsCwbWorker");

    const auto kSampleInterval = std::chrono::milliseconds(100);

    while (mRunning) {
        requestCwbSample();

        auto sleepDuration = (mInitFailures >= 15) ? std::chrono::seconds(5) : kSampleInterval;

        std::unique_lock<std::mutex> lock(mLock);
        mCv.wait_for(lock, sleepDuration, [this] { return !mRunning; });
    }
}
