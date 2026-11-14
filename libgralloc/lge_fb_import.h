/* SPDX-License-Identifier: Apache-2.0 */
#ifndef LGE_OREO_FB_IMPORT_H
#define LGE_OREO_FB_IMPORT_H

#include <errno.h>
#include <limits.h>
#include <new>
#include <stdint.h>
#include <unistd.h>

#include "gralloc_priv.h"
#include "fb_priv.h"

int mapFrameBufferLocked(private_module_t* module);

namespace lge_oreo_fb {

// This gralloc DSO has one HAL_MODULE_INFO_SYM and one primary framebuffer.
// External inline linkage makes this state shared by gpu.cpp and mapper.cpp.
struct References {
    private_module_t* owner;
    uint32_t count[32];
};

inline References& references() {
    static References refs = {};
    return refs;
}

class ModuleLock {
    pthread_mutex_t* mutex;
public:
    explicit ModuleLock(private_module_t* module) : mutex(&module->lock) {
        pthread_mutex_lock(mutex);
    }
    ~ModuleLock() { pthread_mutex_unlock(mutex); }
private:
    ModuleLock(const ModuleLock&);
    ModuleLock& operator=(const ModuleLock&);
};

// Caller holds module->lock. The framebuffer mapping outlives exported handles.
inline int geometryLocked(private_module_t* module, size_t* pageSize) {
    if (!module->framebuffer) {
        const int result = mapFrameBufferLocked(module);
        if (result) return result;
    }
    if (!module->framebuffer || !module->numBuffers || module->numBuffers > 32 ||
            !module->info.bits_per_pixel || module->info.bits_per_pixel % 8 ||
            !module->finfo.line_length || !module->info.yres ||
            module->finfo.line_length % (module->info.bits_per_pixel / 8)) return -EINVAL;
    const uint64_t page = uint64_t(module->finfo.line_length) * module->info.yres;
    const uint64_t total = page * module->numBuffers;
    const uint64_t base = static_cast<uint32_t>(module->framebuffer->base);
    if (!page || page > INT_MAX || page % getpagesize() ||
            total > INT_MAX || total > module->finfo.smem_len ||
            module->framebuffer->size <= 0 ||
            total > static_cast<uint32_t>(module->framebuffer->size) ||
            !base || base + total > (uint64_t(1) << 32)) return -EINVAL;
    References& refs = references();
    if (refs.owner && refs.owner != module) return -EINVAL;
    refs.owner = module;
    *pageSize = static_cast<size_t>(page);
    return 0;
}

inline int indexLocked(private_module_t* module, const private_handle_t* handle,
        uint32_t* index, size_t* pageSize) {
    int result = geometryLocked(module, pageSize);
    if (result) return result;
    if (handle->offset < 0 || handle->size != static_cast<int>(*pageSize) ||
            static_cast<size_t>(handle->offset) % *pageSize ||
            handle->width != static_cast<int>(module->info.xres) ||
            handle->height != static_cast<int>(module->info.yres) ||
            handle->format != static_cast<int>(module->fbFormat)) return -EINVAL;
    *index = static_cast<size_t>(handle->offset) / *pageSize;
    return *index < module->numBuffers ? 0 : -EINVAL;
}

inline int dropLocked(private_module_t* module, const private_handle_t* handle) {
    uint32_t index;
    size_t pageSize;
    int result = indexLocked(module, handle, &index, &pageSize);
    if (result) return result;
    uint32_t& count = references().count[index];
    if (!count) return -EINVAL;
    if (--count == 0) module->bufferMask &= ~(uint32_t(1) << index);
    return 0;
}

inline int allocateLocked(private_module_t* module, buffer_handle_t* outHandle) {
    if (!outHandle) return -EINVAL;
    *outHandle = NULL;
    size_t pageSize;
    int result = geometryLocked(module, &pageSize);
    if (result) return result;
    uint32_t index = 0;
    for (; index < module->numBuffers; ++index) {
        if (!(module->bufferMask & (uint32_t(1) << index))) break;
    }
    if (index == module->numBuffers) return -ENOMEM;
    if (references().count[index]) return -EINVAL;

    const int fd = dup(module->framebuffer->fd);
    if (fd < 0) return -errno;
    // private_handle_t always advertises two FDs. Metadata is unused for FB
    // handles, but its slot must be valid for Oreo's native_handle_clone().
    const int metadataFd = dup(module->framebuffer->fd);
    if (metadataFd < 0) {
        const int error = -errno;
        close(fd);
        return error;
    }
    private_handle_t* handle = new (std::nothrow) private_handle_t(fd, pageSize,
            private_handle_t::PRIV_FLAGS_USES_PMEM |
            private_handle_t::PRIV_FLAGS_FRAMEBUFFER, BUFFER_TYPE_UI,
            module->fbFormat, module->info.xres, module->info.yres, metadataFd);
    if (!handle) {
        close(metadataFd);
        close(fd);
        return -ENOMEM;
    }
    handle->offset = pageSize * index;
    handle->base = static_cast<uint32_t>(module->framebuffer->base) + handle->offset;
    references().count[index] = 1; // allocator owns the original handle
    module->bufferMask |= uint32_t(1) << index;
    *outHandle = handle;
    return 0;
}

inline int importBuffer(private_module_t* module, private_handle_t* handle) {
    ModuleLock lock(module);
    uint32_t index;
    size_t pageSize;
    int result = indexLocked(module, handle, &index, &pageSize);
    if (result) return result;
    uint32_t& count = references().count[index];
    if (count == UINT32_MAX) return -EOVERFLOW;
    ++count;
    module->bufferMask |= uint32_t(1) << index;
    handle->base = static_cast<uint32_t>(module->framebuffer->base) + handle->offset;
    handle->base_metadata = 0; // never map the unused metadata FD alias
    return 0;
}

inline int unregisterBuffer(private_module_t* module, private_handle_t* handle) {
    ModuleLock lock(module);
    const int result = dropLocked(module, handle);
    if (!result) handle->base = handle->base_metadata = 0;
    // The Oreo mapper owns native_handle_close/delete for this imported copy.
    return result;
}

inline int freeAllocation(private_module_t* module, const private_handle_t* handle) {
    ModuleLock lock(module);
    const int result = dropLocked(module, handle);
    if (result) return result;
    // Gralloc0Allocator frees originals after its import callback. Imported
    // copies keep the slot reserved and own their own duplicated FDs.
    close(handle->fd);
    close(handle->fd_metadata);
    delete handle;
    return 0;
}

} // namespace lge_oreo_fb
#endif
