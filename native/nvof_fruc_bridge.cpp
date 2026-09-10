#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>

#include <cuda.h>
#include <NvOFFRUC.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <new>
#include <string>
#include <vector>

namespace {

struct FrucBridge {
    HMODULE module = nullptr;
    CUcontext context = nullptr;
    CUdeviceptr inputs[2] = {};
    CUdeviceptr output = 0;
    NvOFFRUCHandle handle = nullptr;
    PtrToFuncNvOFFRUCCreate create = nullptr;
    PtrToFuncNvOFFRUCRegisterResource register_resource = nullptr;
    PtrToFuncNvOFFRUCUnregisterResource unregister_resource = nullptr;
    PtrToFuncNvOFFRUCProcess process = nullptr;
    PtrToFuncNvOFFRUCDestroy destroy = nullptr;
    int width = 0;
    int height = 0;
    size_t pitch = 0;
    int input_index = 0;
    bool primed = false;
    double previous_timestamp = 0.0;
    std::vector<uint8_t> host_argb;
    std::mutex mutex;
};

void write_error(char* buffer, size_t size, const std::string& message) {
    if (buffer == nullptr || size == 0) {
        return;
    }
    const size_t count = std::min(size - 1, message.size());
    std::memcpy(buffer, message.data(), count);
    buffer[count] = '\0';
}

std::string cuda_error(CUresult result, const char* operation) {
    const char* name = nullptr;
    const char* description = nullptr;
    cuGetErrorName(result, &name);
    cuGetErrorString(result, &description);
    std::string message(operation);
    message += " failed";
    if (name != nullptr) {
        message += " [";
        message += name;
        message += "]";
    }
    if (description != nullptr) {
        message += ": ";
        message += description;
    }
    return message;
}

bool push_context(FrucBridge* bridge, char* error, size_t error_size) {
    const CUresult result = cuCtxPushCurrent(bridge->context);
    if (result != CUDA_SUCCESS) {
        write_error(error, error_size, cuda_error(result, "cuCtxPushCurrent"));
        return false;
    }
    return true;
}

void pop_context() {
    CUcontext popped = nullptr;
    cuCtxPopCurrent(&popped);
}

void cleanup(FrucBridge* bridge) {
    if (bridge == nullptr) {
        return;
    }
    if (bridge->context != nullptr) {
        CUresult pushed = cuCtxPushCurrent(bridge->context);
        if (pushed == CUDA_SUCCESS) {
            if (bridge->handle != nullptr && bridge->unregister_resource != nullptr) {
                NvOFFRUC_UNREGISTER_RESOURCE_PARAM params = {};
                params.pArrResource[0] = &bridge->output;
                params.pArrResource[1] = &bridge->inputs[0];
                params.pArrResource[2] = &bridge->inputs[1];
                params.uiCount = 3;
                bridge->unregister_resource(bridge->handle, &params);
            }
            if (bridge->handle != nullptr && bridge->destroy != nullptr) {
                bridge->destroy(bridge->handle);
                bridge->handle = nullptr;
            }
            for (CUdeviceptr& input : bridge->inputs) {
                if (input != 0) {
                    cuMemFree(input);
                    input = 0;
                }
            }
            if (bridge->output != 0) {
                cuMemFree(bridge->output);
                bridge->output = 0;
            }
            pop_context();
        }
        cuCtxDestroy(bridge->context);
        bridge->context = nullptr;
    }
    if (bridge->module != nullptr) {
        FreeLibrary(bridge->module);
        bridge->module = nullptr;
    }
}

template <typename T>
T load_symbol(HMODULE module, const char* name) {
    return reinterpret_cast<T>(GetProcAddress(module, name));
}

}  // namespace

extern "C" __declspec(dllexport) void* nvof_fruc_create(
    const wchar_t* fruc_dll_path,
    int width,
    int height,
    int device_id,
    char* error,
    size_t error_size) {
    if (fruc_dll_path == nullptr || width <= 0 || height <= 0 || device_id < 0) {
        write_error(error, error_size, "invalid create parameters");
        return nullptr;
    }

    FrucBridge* bridge = new (std::nothrow) FrucBridge();
    if (bridge == nullptr) {
        write_error(error, error_size, "unable to allocate bridge state");
        return nullptr;
    }
    bridge->width = width;
    bridge->height = height;
    bridge->pitch = static_cast<size_t>(width) * 4;
    bridge->host_argb.resize(bridge->pitch * static_cast<size_t>(height));

    CUresult cuda_status = cuInit(0);
    if (cuda_status != CUDA_SUCCESS) {
        write_error(error, error_size, cuda_error(cuda_status, "cuInit"));
        delete bridge;
        return nullptr;
    }
    CUdevice device = 0;
    cuda_status = cuDeviceGet(&device, device_id);
    if (cuda_status != CUDA_SUCCESS) {
        write_error(error, error_size, cuda_error(cuda_status, "cuDeviceGet"));
        delete bridge;
        return nullptr;
    }
    cuda_status = cuCtxCreate(&bridge->context, 0, device);
    if (cuda_status != CUDA_SUCCESS) {
        write_error(error, error_size, cuda_error(cuda_status, "cuCtxCreate"));
        delete bridge;
        return nullptr;
    }

    bridge->module = LoadLibraryExW(
        fruc_dll_path,
        nullptr,
        LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR | LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);
    if (bridge->module == nullptr) {
        const DWORD code = GetLastError();
        write_error(error, error_size, "LoadLibraryW(NvOFFRUC.dll) failed with Win32 error " + std::to_string(code));
        pop_context();
        cleanup(bridge);
        delete bridge;
        return nullptr;
    }
    bridge->create = load_symbol<PtrToFuncNvOFFRUCCreate>(bridge->module, CreateProcName);
    bridge->register_resource = load_symbol<PtrToFuncNvOFFRUCRegisterResource>(bridge->module, RegisterResourceProcName);
    bridge->unregister_resource = load_symbol<PtrToFuncNvOFFRUCUnregisterResource>(bridge->module, UnregisterResourceProcName);
    bridge->process = load_symbol<PtrToFuncNvOFFRUCProcess>(bridge->module, ProcessProcName);
    bridge->destroy = load_symbol<PtrToFuncNvOFFRUCDestroy>(bridge->module, DestroyProcName);
    if (bridge->create == nullptr || bridge->register_resource == nullptr ||
        bridge->unregister_resource == nullptr || bridge->process == nullptr || bridge->destroy == nullptr) {
        write_error(error, error_size, "NvOFFRUC.dll is missing one or more required exports");
        pop_context();
        cleanup(bridge);
        delete bridge;
        return nullptr;
    }

    const size_t bytes = bridge->pitch * static_cast<size_t>(height);
    for (CUdeviceptr& input : bridge->inputs) {
        cuda_status = cuMemAlloc(&input, bytes);
        if (cuda_status != CUDA_SUCCESS) {
            write_error(error, error_size, cuda_error(cuda_status, "cuMemAlloc(input)"));
            pop_context();
            cleanup(bridge);
            delete bridge;
            return nullptr;
        }
    }
    cuda_status = cuMemAlloc(&bridge->output, bytes);
    if (cuda_status != CUDA_SUCCESS) {
        write_error(error, error_size, cuda_error(cuda_status, "cuMemAlloc(output)"));
        pop_context();
        cleanup(bridge);
        delete bridge;
        return nullptr;
    }

    NvOFFRUC_CREATE_PARAM create_params = {};
    create_params.uiWidth = static_cast<uint32_t>(width);
    create_params.uiHeight = static_cast<uint32_t>(height);
    create_params.pDevice = nullptr;
    create_params.eResourceType = CudaResource;
    create_params.eSurfaceFormat = ARGBSurface;
    create_params.eCUDAResourceType = CudaResourceCuDevicePtr;
    NvOFFRUC_STATUS status = bridge->create(&create_params, &bridge->handle);
    if (status != NvOFFRUC_SUCCESS) {
        write_error(error, error_size, "NvOFFRUCCreate failed with status " + std::to_string(status));
        pop_context();
        cleanup(bridge);
        delete bridge;
        return nullptr;
    }

    NvOFFRUC_REGISTER_RESOURCE_PARAM resources = {};
    resources.pArrResource[0] = &bridge->output;
    resources.pArrResource[1] = &bridge->inputs[0];
    resources.pArrResource[2] = &bridge->inputs[1];
    resources.uiCount = 3;
    status = bridge->register_resource(bridge->handle, &resources);
    if (status != NvOFFRUC_SUCCESS) {
        write_error(error, error_size, "NvOFFRUCRegisterResource failed with status " + std::to_string(status));
        pop_context();
        cleanup(bridge);
        delete bridge;
        return nullptr;
    }

    pop_context();
    write_error(error, error_size, "");
    return bridge;
}

// Return values: -1=failure, 0=first frame primed, 1=interpolated frame produced.
extern "C" __declspec(dllexport) int nvof_fruc_push_bgr(
    void* opaque,
    const uint8_t* source_bgr,
    size_t source_stride,
    double timestamp,
    uint8_t* destination_bgr,
    size_t destination_stride,
    int* repeated,
    double* elapsed_ms,
    char* error,
    size_t error_size) {
    FrucBridge* bridge = static_cast<FrucBridge*>(opaque);
    if (bridge == nullptr || source_bgr == nullptr || destination_bgr == nullptr ||
        source_stride < static_cast<size_t>(bridge->width) * 3 ||
        destination_stride < static_cast<size_t>(bridge->width) * 3) {
        write_error(error, error_size, "invalid frame parameters");
        return -1;
    }
    std::lock_guard<std::mutex> guard(bridge->mutex);
    if (!push_context(bridge, error, error_size)) {
        return -1;
    }

    for (int y = 0; y < bridge->height; ++y) {
        const uint8_t* source = source_bgr + static_cast<size_t>(y) * source_stride;
        uint8_t* target = bridge->host_argb.data() + static_cast<size_t>(y) * bridge->pitch;
        for (int x = 0; x < bridge->width; ++x) {
            target[x * 4 + 0] = source[x * 3 + 2];
            target[x * 4 + 1] = source[x * 3 + 1];
            target[x * 4 + 2] = source[x * 3 + 0];
            target[x * 4 + 3] = 255;
        }
    }

    CUdeviceptr input = bridge->inputs[bridge->input_index];
    CUresult cuda_status = cuMemcpyHtoD(input, bridge->host_argb.data(), bridge->host_argb.size());
    if (cuda_status != CUDA_SUCCESS) {
        write_error(error, error_size, cuda_error(cuda_status, "cuMemcpyHtoD"));
        pop_context();
        return -1;
    }

    if (bridge->primed && timestamp <= bridge->previous_timestamp) {
        timestamp = bridge->previous_timestamp + 0.000001;
    }
    bool frame_repeated = false;
    NvOFFRUC_PROCESS_IN_PARAMS in_params = {};
    NvOFFRUC_PROCESS_OUT_PARAMS out_params = {};
    in_params.stFrameDataInput.pFrame = &bridge->inputs[bridge->input_index];
    in_params.stFrameDataInput.nTimeStamp = timestamp;
    in_params.stFrameDataInput.nCuSurfacePitch = bridge->pitch;
    out_params.stFrameDataOutput.pFrame = &bridge->output;
    out_params.stFrameDataOutput.nTimeStamp = bridge->primed
        ? (bridge->previous_timestamp + timestamp) * 0.5
        : timestamp;
    out_params.stFrameDataOutput.nCuSurfacePitch = bridge->pitch;
    out_params.stFrameDataOutput.bHasFrameRepetitionOccurred = &frame_repeated;

    const auto started = std::chrono::steady_clock::now();
    const NvOFFRUC_STATUS status = bridge->process(bridge->handle, &in_params, &out_params);
    const auto finished = std::chrono::steady_clock::now();
    if (elapsed_ms != nullptr) {
        *elapsed_ms = std::chrono::duration<double, std::milli>(finished - started).count();
    }
    if (status != NvOFFRUC_SUCCESS) {
        write_error(error, error_size, "NvOFFRUCProcess failed with status " + std::to_string(status));
        pop_context();
        return -1;
    }

    const bool had_previous = bridge->primed;
    bridge->primed = true;
    bridge->previous_timestamp = timestamp;
    bridge->input_index = (bridge->input_index + 1) % 2;
    if (!had_previous) {
        if (repeated != nullptr) {
            *repeated = frame_repeated ? 1 : 0;
        }
        pop_context();
        write_error(error, error_size, "");
        return 0;
    }

    cuda_status = cuMemcpyDtoH(bridge->host_argb.data(), bridge->output, bridge->host_argb.size());
    if (cuda_status != CUDA_SUCCESS) {
        write_error(error, error_size, cuda_error(cuda_status, "cuMemcpyDtoH"));
        pop_context();
        return -1;
    }
    for (int y = 0; y < bridge->height; ++y) {
        const uint8_t* source = bridge->host_argb.data() + static_cast<size_t>(y) * bridge->pitch;
        uint8_t* target = destination_bgr + static_cast<size_t>(y) * destination_stride;
        for (int x = 0; x < bridge->width; ++x) {
            target[x * 3 + 0] = source[x * 4 + 2];
            target[x * 3 + 1] = source[x * 4 + 1];
            target[x * 3 + 2] = source[x * 4 + 0];
        }
    }
    if (repeated != nullptr) {
        *repeated = frame_repeated ? 1 : 0;
    }
    pop_context();
    write_error(error, error_size, "");
    return 1;
}

extern "C" __declspec(dllexport) void nvof_fruc_destroy(void* opaque) {
    FrucBridge* bridge = static_cast<FrucBridge*>(opaque);
    if (bridge == nullptr) {
        return;
    }
    {
        std::lock_guard<std::mutex> guard(bridge->mutex);
        cleanup(bridge);
    }
    delete bridge;
}
