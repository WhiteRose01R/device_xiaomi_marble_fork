/*
 * Copyright (C) 2024-2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "udals_daemon"

#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/parseint.h>
#include <android-base/strings.h>
#include <android/hardware_buffer.h>
#include <vndk/hardware_buffer.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <mutex>
#include <string>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

constexpr const char* kBacklightPath =
    "/sys/class/backlight/panel0-backlight/brightness";
constexpr const char* kUdalsDeviceNode = "/dev/udals_rgb";

// TCS3701 under-display sensor aperture on marble: center (745, 51), radius 47
constexpr int32_t kRoiLeft = 698;
constexpr int32_t kRoiTop = 4;
constexpr int32_t kRoiRight = 792;
constexpr int32_t kRoiBottom = 98;
constexpr int32_t kRoiWidth = kRoiRight - kRoiLeft;   // 94
constexpr int32_t kRoiHeight = kRoiBottom - kRoiTop;  // 94

// Relative center inside the 94x94 cropped buffer
constexpr int32_t kCenterRelX = 47;
constexpr int32_t kCenterRelY = 47;
constexpr int32_t kRadius1 = 18;
constexpr int32_t kRadius2 = 34;
constexpr int32_t kRadius3 = 47;

volatile std::sig_atomic_t gRunning = 1;

void signalHandler(int /* signum */) {
    gRunning = 0;
}

uint32_t readBacklight() {
    std::string content;
    if (android::base::ReadFileToString(kBacklightPath, &content)) {
        uint32_t bkl = 0;
        if (android::base::ParseUint(android::base::Trim(content), &bkl)) {
            return bkl;
        }
    }
    return 0;
}

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

// Complete Qualcomm DisplayConfig ClientInterface definition matching libdisplayconfig.qti.so vtable
class ClientInterface {
public:
    // [0]
    virtual int IsDisplayConnected(DisplayType type, bool* connected) = 0;
    // [1]
    virtual int SetDisplayStatus(DisplayType type, ExternalStatus status) = 0;
    // [2]
    virtual int ConfigureDynRefreshRate(DynRefreshRateOp op, uint32_t hz) = 0;
    // [3]
    virtual int GetConfigCount(DisplayType type, uint32_t* count) = 0;
    // [4]
    virtual int GetActiveConfig(DisplayType type, uint32_t* index) = 0;
    // [5]
    virtual int SetActiveConfig(DisplayType type, uint32_t index) = 0;
    // [6]
    virtual int GetDisplayAttributes(uint32_t index, DisplayType type, void* attr) = 0;
    // [7]
    virtual int SetPanelBrightness(uint32_t level) = 0;
    // [8]
    virtual int GetPanelBrightness(uint32_t* level) = 0;
    // [9]
    virtual int MinHdcpEncryptionLevelChanged(DisplayType type, uint32_t min_enc_level) = 0;
    // [10]
    virtual int RefreshScreen() = 0;
    // [11]
    virtual int ControlPartialUpdate(DisplayType type, bool enable) = 0;
    // [12]
    virtual int ToggleScreenUpdate(bool on) = 0;
    // [13]
    virtual int SetIdleTimeout(uint32_t value) = 0;
    // [14]
    virtual int GetHDRCapabilities(DisplayType type, void* caps) = 0;
    // [15]
    virtual int SetCameraLaunchStatus(uint32_t on) = 0;
    // [16] Destructor
    virtual ~ClientInterface() = default;
    // [17]
    virtual int DisplayBWTransactionPending(bool* status) = 0;
    // [18]
    virtual int SetDisplayAnimating(uint64_t display_id, bool animating) = 0;
    // [19]
    virtual int ControlIdlePowerCollapse(bool enable, bool synchronous) = 0;
    // [20]
    virtual int GetWriteBackCapabilities(bool* status) = 0;
    // [21]
    virtual int SetDisplayDppsAdROI(uint32_t display_id, uint32_t h_start, uint32_t h_end,
                                   uint32_t v_start, uint32_t v_end, uint32_t factor_in,
                                   uint32_t factor_out) = 0;
    // [22]
    virtual int UpdateVSyncSourceOnPowerModeOff() = 0;
    // [23]
    virtual int UpdateVSyncSourceOnPowerModeDoze() = 0;
    // [24]
    virtual int SetPowerMode(uint32_t display_id, uint32_t power_mode) = 0;
    // [25]
    virtual int IsPowerModeOverrideSupported(uint32_t display_id, bool* supported) = 0;
    // [26]
    virtual int IsHDRSupported(uint32_t display_id, bool* supported) = 0;
    // [27]
    virtual int IsWCGSupported(uint32_t display_id, bool* supported) = 0;
    // [28]
    virtual int SetLayerAsMask(uint32_t display_id, uint64_t layer_id) = 0;
    // [29]
    virtual int GetDebugProperty(std::string prop_name, std::string* value) = 0;
    // [30]
    virtual int GetActiveBuiltinDisplayAttributes(void* attr) = 0;
    // [31]
    virtual int SetPanelLuminanceAttributes(uint32_t display_id, float min_lum, float max_lum) = 0;
    // [32]
    virtual int IsBuiltInDisplay(uint32_t display_id, bool* is_builtin) = 0;
    // [33]
    virtual int IsAsyncVDSCreationSupported(bool* supported) = 0;
    // [34]
    virtual int CreateVirtualDisplay(uint32_t width, uint32_t height, int format) = 0;
    // [35]
    virtual int GetSupportedDSIBitClks(uint32_t display_id, std::vector<uint64_t>* bit_clks) = 0;
    // [36]
    virtual int GetDSIClk(uint32_t display_id, uint64_t* bit_clk) = 0;
    // [37]
    virtual int SetDSIClk(uint32_t display_id, uint64_t bit_clk) = 0;
    // [38]
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
        mCv.notify_one();
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

private:
    std::mutex mMutex;
    std::condition_variable mCv;
    bool mDone = false;
    int32_t mError = 0;
};

int main() {
    std::signal(SIGTERM, signalHandler);
    std::signal(SIGINT, signalHandler);

    // Drop privileges: prevent acquiring new privileges via execve
    prctl(PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0);

    LOG(INFO) << "Starting UDALS aperture sampling daemon (hardware CWB)";

    // Precompute sRGB gamma lookup table (c / 255.0)^2.2
    std::array<float, 256> gammaLut{};
    for (size_t i = 0; i < 256; ++i) {
        gammaLut[i] = std::pow(static_cast<float>(i) / 255.0f, 2.2f);
    }

    // Attempt to dynamically load Qualcomm DisplayConfig CWB interface
    using CreateFn = int (*)(std::string, DisplayConfig::ConfigCallback*,
                             DisplayConfig::ClientInterface**);
    using DestroyFn = void (*)(DisplayConfig::ClientInterface*);
    using SetCwbDirectFn = int (*)(void*, uint32_t, DisplayConfig::Rect, bool,
                                   const native_handle_t*);

    void* dpyConfigLib = dlopen("libdisplayconfig.qti.so", RTLD_NOW);
    CreateFn createClient = nullptr;
    DestroyFn destroyClient = nullptr;
    SetCwbDirectFn setCwbDirect = nullptr;

    if (dpyConfigLib != nullptr) {
        createClient = reinterpret_cast<CreateFn>(
            dlsym(dpyConfigLib,
                  "_ZN13DisplayConfig15ClientInterface6CreateENSt3__112basic_stringIcNS1_11char_traitsIcEENS1_9allocatorIcEEEEPNS_14ConfigCallbackEPPS0_"));
        destroyClient = reinterpret_cast<DestroyFn>(
            dlsym(dpyConfigLib, "_ZN13DisplayConfig15ClientInterface7DestroyEPS0_"));
        setCwbDirect = reinterpret_cast<SetCwbDirectFn>(
            dlsym(dpyConfigLib,
                  "_ZN13DisplayConfig10ClientImpl18SetCWBOutputBufferEjNS_4RectEbPK13native_handle"));
    }

    CwbCallbackHandler cwbCallback;
    DisplayConfig::ClientInterface* cwbClient = nullptr;
    AHardwareBuffer* cwbBuffer = nullptr;
    const native_handle_t* cwbBufferHandle = nullptr;
    uint32_t bufferStride = kRoiWidth;

    for (int retry = 0; retry < 5 && gRunning; ++retry) {
        if (createClient != nullptr && cwbClient == nullptr) {
            createClient("udals_daemon", &cwbCallback, &cwbClient);
        }
        if (cwbClient != nullptr && cwbBufferHandle == nullptr) {
            AHardwareBuffer_Desc desc = {
                .width = static_cast<uint32_t>(kRoiWidth),
                .height = static_cast<uint32_t>(kRoiHeight),
                .layers = 1,
                .format = AHARDWAREBUFFER_FORMAT_R8G8B8A8_UNORM,
                .usage = AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN |
                         AHARDWAREBUFFER_USAGE_CPU_WRITE_OFTEN |
                         AHARDWAREBUFFER_USAGE_GPU_COLOR_OUTPUT,
                .stride = 0,
                .rfu0 = 0,
                .rfu1 = 0,
            };
            if (AHardwareBuffer_allocate(&desc, &cwbBuffer) == 0 && cwbBuffer != nullptr) {
                AHardwareBuffer_describe(cwbBuffer, &desc);
                cwbBufferHandle = AHardwareBuffer_getNativeHandle(cwbBuffer);
                bufferStride = desc.stride;
                LOG(INFO) << "Hardware CWB buffer allocated: " << kRoiWidth << "x"
                          << kRoiHeight << " RGBA, stride=" << bufferStride;
                break;
            }
            LOG(WARNING) << "Failed to allocate 94x94 CWB buffer";
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (cwbClient == nullptr || cwbBufferHandle == nullptr) {
        LOG(WARNING) << "Hardware CWB client unavailable; sleeping to conserve power";
        int outFd = open(kUdalsDeviceNode, O_RDWR | O_CLOEXEC);
        if (outFd >= 0) {
            float zeroRgb[3] = {0.0f, 0.0f, 0.0f};
            pwrite(outFd, zeroRgb, sizeof(zeroRgb), 0);
            close(outFd);
        }
        if (cwbClient != nullptr && destroyClient != nullptr) {
            destroyClient(cwbClient);
        }
        if (dpyConfigLib != nullptr) {
            dlclose(dpyConfigLib);
        }
        while (gRunning) {
            pause();
        }
        return 0;
    }

    DisplayConfig::Rect roiRect = {
        .left = kRoiLeft,
        .top = kRoiTop,
        .right = kRoiRight,
        .bottom = kRoiBottom,
    };

    int outFd = -1;
    float lastRgb[3] = {0.0f, 0.0f, 0.0f};

    while (gRunning) {
        if (outFd < 0) {
            outFd = open(kUdalsDeviceNode, O_RDWR | O_CLOEXEC);
        }

        uint32_t brightness = readBacklight();

        if (brightness == 0) {
            // Display is off: emit zero bleed and sleep
            lastRgb[0] = 0.0f;
            lastRgb[1] = 0.0f;
            lastRgb[2] = 0.0f;
            if (outFd >= 0) {
                pwrite(outFd, lastRgb, sizeof(lastRgb), 0);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            continue;
        }

        if (cwbClient != nullptr && cwbBufferHandle != nullptr) {
            int ret = -1;
            if (setCwbDirect != nullptr) {
                ret = setCwbDirect(cwbClient, 0, roiRect, true, cwbBufferHandle);
            } else {
                ret = cwbClient->SetCWBOutputBuffer(0, roiRect, true, cwbBufferHandle);
            }

            int32_t cwbError = 0;
            if (ret == 0 &&
                cwbCallback.waitForDone(std::chrono::milliseconds(80), &cwbError) &&
                cwbError == 0) {
                void* vaddr = nullptr;
                if (AHardwareBuffer_lock(cwbBuffer, AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN |
                                                     AHARDWAREBUFFER_USAGE_CPU_WRITE_OFTEN,
                                         -1, nullptr, &vaddr) == 0 &&
                    vaddr != nullptr) {
                    const uint8_t* pixels = static_cast<const uint8_t*>(vaddr);
                    float weightedR = 0.0f;
                    float weightedG = 0.0f;
                    float weightedB = 0.0f;
                    float totalWeight = 0.0f;

                    for (int32_t y = 0; y < kRoiHeight; ++y) {
                        int32_t dy = y - kCenterRelY;
                        int32_t dy2 = dy * dy;
                        const uint8_t* row = pixels + (y * bufferStride * 4);

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

                            weightedR += gammaLut[r] * w;
                            weightedG += gammaLut[g] * w;
                            weightedB += gammaLut[b] * w;
                            totalWeight += w;
                        }
                    }

                    // Clear buffer after processing
                    std::memset(vaddr, 0, bufferStride * kRoiHeight * 4);
                    AHardwareBuffer_unlock(cwbBuffer, nullptr);

                    if (totalWeight > 0.0f) {
                        lastRgb[0] = std::clamp(weightedR / totalWeight, 0.0f, 1.0f);
                        lastRgb[1] = std::clamp(weightedG / totalWeight, 0.0f, 1.0f);
                        lastRgb[2] = std::clamp(weightedB / totalWeight, 0.0f, 1.0f);
                    }
                }
            }

            // Update sample or heartbeat
            if (outFd >= 0) {
                pwrite(outFd, lastRgb, sizeof(lastRgb), 0);
            }
        } else {
            // Write zero bleed if CWB is inactive
            if (outFd >= 0) {
                pwrite(outFd, lastRgb, sizeof(lastRgb), 0);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (cwbBuffer != nullptr) {
        AHardwareBuffer_release(cwbBuffer);
    }
    if (cwbClient != nullptr && destroyClient != nullptr) {
        destroyClient(cwbClient);
    }
    if (dpyConfigLib != nullptr) {
        dlclose(dpyConfigLib);
    }
    if (outFd >= 0) {
        close(outFd);
    }

    LOG(INFO) << "UDALS aperture sampling daemon stopped cleanly";
    return 0;
}
