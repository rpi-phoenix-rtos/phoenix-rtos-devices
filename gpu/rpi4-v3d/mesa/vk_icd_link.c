/*
 * vk_icd_link.c — loader-less ICD linkage for V3DV on Phoenix (Vulkan Tier 0).
 *
 * vkQuake (and our harness) calls Vulkan through vkGetInstanceProcAddr; it does NOT
 * use the Khronos loader (libvulkan.so / ICD JSON / layers). We statically link the
 * V3DV ICD and resolve the single bootstrap symbol directly:
 *     vkGetInstanceProcAddr -> v3dv_GetInstanceProcAddr (the generated dispatch root).
 * Everything else is fetched through that table at runtime, so no other vk* alias is
 * needed (mirrors how the GL build linked libGL-phoenix.a without a GLX/EGL loader).
 *
 * Also: WSI stubs. We bypass the standard wsi_common_drm (no /dev/dri display node,
 * no PRIME) and will build a custom /dev/fb0 swapchain at Tier 4. For Tier 0 the
 * swapchain/surface entrypoints and the wsi_* hooks v3dv_device.c references must
 * still resolve -> stub them here (the link-drive populates the exact set).
 *
 * Copyright 2026 Phoenix Systems  %LICENSE%
 */
#include <stdbool.h>
#include <vulkan/vulkan.h>

/* The generated V3DV dispatch root (v3dv_entrypoints.c). */
PFN_vkVoidFunction v3dv_GetInstanceProcAddr(VkInstance instance, const char *pName);

/* The public bootstrap symbol the loader-less client links against. */
PFN_vkVoidFunction vkGetInstanceProcAddr(VkInstance instance, const char *pName)
{
	return v3dv_GetInstanceProcAddr(instance, pName);
}

/* NOTE: vk_icdGetInstanceProcAddr is defined by V3DV itself (v3dv_device.c); we do NOT
 * redefine it here (it would multiply-define). Only the bare vkGetInstanceProcAddr
 * bootstrap (which V3DV does not export) is provided above. */

/* ---------------------------------------------------------------------------
 * WSI bypass stubs (Tier 0).
 *
 * Phoenix has no wsi_common_drm (no /dev/dri display node, no PRIME). The custom
 * /dev/fb0 swapchain is Tier 4. For Tier 0 the WSI symbols v3dv_device.c references
 * must link; the swapchain/surface entrypoints they would register are simply absent
 * (reported unsupported) until the fb0 swapchain lands.
 *
 * The three wsi_*_entrypoints tables are passed to vk_*_dispatch_table_from_entrypoints
 * with overwrite=false, so an all-zero table is correct: every WSI entrypoint reads as
 * "unsupported" and the V3DV-native + vk_common entrypoints win. The exact struct types
 * are large generated dispatch tables; we only need storage of the right linkage, so we
 * define them as zero-initialized opaque blobs of ample size and alias the symbols.
 * ------------------------------------------------------------------------- */

/* Opaque zero-filled storage for the three dispatch tables. The real structs are
 * arrays of function pointers; 4 KiB of zeros is far larger than any of them, so every
 * slot reads NULL (= unsupported). Named via asm() to match the C symbol the tables
 * are referenced by. */
static const unsigned char wsi_entrypoint_zeros[4096] = {0};

extern const unsigned char wsi_instance_entrypoints[]
	__attribute__((alias("wsi_entrypoint_zeros")));
extern const unsigned char wsi_physical_device_entrypoints[]
	__attribute__((alias("wsi_entrypoint_zeros")));
extern const unsigned char wsi_device_entrypoints[]
	__attribute__((alias("wsi_entrypoint_zeros")));

/* v3dv_wsi_init/finish: no WSI device set up for Tier 0. Init returns VK_SUCCESS so
 * physical-device creation proceeds; finish is a no-op. */
typedef struct v3dv_physical_device v3dv_physical_device;
VkResult v3dv_wsi_init(v3dv_physical_device *pd)
{
	(void)pd;
	return VK_SUCCESS;
}

void v3dv_wsi_finish(v3dv_physical_device *pd)
{
	(void)pd;
}

/* wsi_common helpers referenced from V3DV meta/image paths but only reached through a
 * live swapchain (none at Tier 0). Trap if ever called before the fb0 swapchain exists. */
struct wsi_device;
VkDeviceMemory wsi_common_get_memory(VkSwapchainKHR swapchain, uint32_t index)
{
	(void)swapchain; (void)index;
	return VK_NULL_HANDLE;
}

VkResult wsi_common_create_swapchain_image(const struct wsi_device *wsi,
                                           const void *create_info,
                                           void *handle, void *out)
{
	(void)wsi; (void)create_info; (void)handle; (void)out;
	return VK_ERROR_FEATURE_NOT_PRESENT;
}

bool wsi_instance_supports_google_display_timing(void *instance)
{
	(void)instance;
	return false;
}
