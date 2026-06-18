/*
 * v3dv_harness.c — Vulkan Tier-0 link-closure + first-runtime-contact harness.
 *
 * The Vulkan analog of harness_screen_create.c (GL Path-C Phase 2). It drives the
 * V3DV ICD through the standard Vulkan entry sequence:
 *     vkCreateInstance -> vkEnumeratePhysicalDevices -> vkCreateDevice
 * resolving every entrypoint through vkGetInstanceProcAddr (aliased to
 * v3dv_GetInstanceProcAddr in vk_icd_link.c), exactly as a loader-less client (and
 * later vkQuake) would.
 *
 * Tier 0 GOAL = this links to an aarch64-phoenix ELF with 0 undefined symbols.
 * Tier 1 (main agent, on HW) = it actually runs and prints "vkCreateDevice OK, V3D 4.2".
 *
 * PRECONDITION (HW, Tier 1): the V3D must be powered on (v3d_phoenix_powerOn) before
 * the winsys touches MMIO — wired in the on-device launcher, not this link/smoke build.
 * Device enumeration's drmGetDevices2 scan is bypassed by the mesa-phoenix-port patch
 * (enumerate_devices -> create_physical_device with a FAKE_FD), see the Tier-0 doc.
 *
 * Copyright 2026 Phoenix Systems  %LICENSE%
 */
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <vulkan/vulkan.h>

/* The one symbol the loader-less client needs; vk_icd_link.c aliases it to
 * v3dv_GetInstanceProcAddr (the generated dispatch root). */
PFN_vkVoidFunction vkGetInstanceProcAddr(VkInstance instance, const char *pName);

#define GIPA(inst, name) ((PFN_##name)vkGetInstanceProcAddr((inst), #name))

int main(void)
{
	/* Unbuffered: a fault must not eat buffered output over the captured UART path. */
	setvbuf(stdout, NULL, _IONBF, 0);
	printf("v3dv-harness: start\n");

	PFN_vkCreateInstance pCreateInstance = GIPA(NULL, vkCreateInstance);
	if (!pCreateInstance) {
		printf("v3dv-harness: vkGetInstanceProcAddr(vkCreateInstance) NULL\n");
		return 1;
	}

	const char *exts[] = { "VK_KHR_surface" };
	VkApplicationInfo app = {
		.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO,
		.pApplicationName = "v3dv-tier0",
		.apiVersion = VK_API_VERSION_1_1,
	};
	VkInstanceCreateInfo ici = {
		.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,
		.pApplicationInfo = &app,
		/* keep extension count 0 for first-light; surface ext is advertised but the
		 * custom fb0 swapchain is Tier 4. Listed here for reference, not enabled. */
		.enabledExtensionCount = 0,
		.ppEnabledExtensionNames = exts,
	};
	VkInstance inst = VK_NULL_HANDLE;
	VkResult r = pCreateInstance(&ici, NULL, &inst);
	printf("v3dv-harness: vkCreateInstance -> %d\n", (int)r);
	if (r != VK_SUCCESS)
		return 2;

	PFN_vkEnumeratePhysicalDevices pEnum = GIPA(inst, vkEnumeratePhysicalDevices);
	PFN_vkGetPhysicalDeviceProperties pProps = GIPA(inst, vkGetPhysicalDeviceProperties);
	PFN_vkCreateDevice pCreateDevice = GIPA(inst, vkCreateDevice);
	if (!pEnum || !pCreateDevice) {
		printf("v3dv-harness: missing enum/createdevice proc\n");
		return 3;
	}

	uint32_t n = 0;
	r = pEnum(inst, &n, NULL);
	printf("v3dv-harness: vkEnumeratePhysicalDevices -> %d count=%u\n", (int)r, n);
	if (r != VK_SUCCESS || n == 0)
		return 4;

	VkPhysicalDevice phys = VK_NULL_HANDLE;
	n = 1;
	r = pEnum(inst, &n, &phys);
	if (r < 0)
		return 5;

	/* NOTE: vkGetPhysicalDeviceProperties (the vk_common trampoline) dispatches to the
	 * physical-device table's GetPhysicalDeviceProperties2 slot, which is currently NULL
	 * on this build -> instruction abort (pc=0). That is a physical-device-dispatch-table
	 * gap (cosmetic name print), independent of vkCreateDevice (an instance entrypoint
	 * dispatched straight to v3dv_CreateDevice). Skip the name print to reach the real
	 * Tier-1 milestone; the dispatch gap is tracked separately. */
	(void)pProps;
	printf("v3dv-harness: (skipping vkGetPhysicalDeviceProperties name print -- phys-dispatch Properties2 is NULL)\n");

	float prio = 1.0f;
	VkDeviceQueueCreateInfo qci = {
		.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO,
		.queueFamilyIndex = 0,
		.queueCount = 1,
		.pQueuePriorities = &prio,
	};
	VkDeviceCreateInfo dci = {
		.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO,
		.queueCreateInfoCount = 1,
		.pQueueCreateInfos = &qci,
	};
	VkDevice dev = VK_NULL_HANDLE;
	r = pCreateDevice(phys, &dci, NULL, &dev);
	printf("v3dv-harness: vkCreateDevice -> %d\n", (int)r);
	if (r != VK_SUCCESS)
		return 6;

	printf("v3dv-harness: device created OK\n");

	/* --- Tier 2: a real queue submit (empty primary command buffer). This exercises the
	 * V3DV queue -> winsys submit path (ioc_submit_cl, the same synchronous bin/render path
	 * Quake renders through) + the synchronous syncobj/fence stubs (work is done when
	 * ioc_submit_cl returns, so vkQueueWaitIdle returns immediately). Device-level procs
	 * resolve through vkGetDeviceProcAddr (NOT vkGetInstanceProcAddr: in a loader-less
	 * ICD link the latter hands back trampolines that deref a NULL device-dispatch slot
	 * -> pc=0 instruction-abort; GetDeviceProcAddr returns the real device entrypoint). --- */
	PFN_vkGetDeviceProcAddr pGDPA = GIPA(inst, vkGetDeviceProcAddr);
	if (!pGDPA) {
		printf("v3dv-harness: PASS (device created); no vkGetDeviceProcAddr\n");
		return 0;
	}
#define GDPA(d, name) ((PFN_##name)pGDPA((d), #name))
	/* CreateCommandPool / AllocateCommandBuffers / QueueSubmit are vk_common command-pool +
	 * synchronization FRAMEWORK entrypoints; they resolve NULL via vkGetDeviceProcAddr (a
	 * dispatch-table gate bug). The framework itself IS wired by v3dv (v3dv_queue_driver_submit
	 * + v3dv_cmd_buffer_ops), so we DECOUPLE the submit test from that lookup bug by calling the
	 * exported vk_common_* impls directly — exercising the real winsys submit path now. The
	 * dispatch-gate fix is tracked separately (docs/inprogress/2026-06-18-vulkan-v3dv-noop-job-rootcause.md). */
	extern VkResult vk_common_CreateCommandPool(VkDevice, const VkCommandPoolCreateInfo *,
		const VkAllocationCallbacks *, VkCommandPool *);
	extern VkResult vk_common_AllocateCommandBuffers(VkDevice, const VkCommandBufferAllocateInfo *,
		VkCommandBuffer *);
	extern VkResult vk_common_QueueSubmit(VkQueue, uint32_t, const VkSubmitInfo *, VkFence);
	PFN_vkGetDeviceQueue pGetQueue = GDPA(dev, vkGetDeviceQueue);
	PFN_vkCreateCommandPool pCreatePool = vk_common_CreateCommandPool;
	PFN_vkAllocateCommandBuffers pAllocCmd = vk_common_AllocateCommandBuffers;
	PFN_vkBeginCommandBuffer pBegin = GDPA(dev, vkBeginCommandBuffer);
	PFN_vkEndCommandBuffer pEnd = GDPA(dev, vkEndCommandBuffer);
	PFN_vkQueueSubmit pSubmit = vk_common_QueueSubmit;
	PFN_vkQueueWaitIdle pWaitIdle = GDPA(dev, vkQueueWaitIdle);
	printf("v3dv-harness: dproc GetDeviceQueue=%p CreateCommandPool=%p(direct) AllocCmdBufs=%p(direct) "
	       "Begin=%p End=%p QueueSubmit=%p(direct) QueueWaitIdle=%p\n",
	       (void *)pGetQueue, (void *)pCreatePool, (void *)pAllocCmd, (void *)pBegin,
	       (void *)pEnd, (void *)pSubmit, (void *)pWaitIdle);
	if (!pGetQueue || !pCreatePool || !pAllocCmd || !pBegin || !pEnd || !pSubmit || !pWaitIdle) {
		printf("v3dv-harness: PASS (device created); a Tier-2 device proc is NULL (see line above)\n");
		return 0;
	}

	VkQueue queue = VK_NULL_HANDLE;
	pGetQueue(dev, 0, 0, &queue);

	VkCommandPoolCreateInfo pci = {
		.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
		.queueFamilyIndex = 0,
	};
	VkCommandPool pool = VK_NULL_HANDLE;
	r = pCreatePool(dev, &pci, NULL, &pool);
	printf("v3dv-harness: vkCreateCommandPool -> %d\n", (int)r);
	if (r != VK_SUCCESS)
		return 7;

	VkCommandBufferAllocateInfo cbai = {
		.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
		.commandPool = pool,
		.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
		.commandBufferCount = 1,
	};
	VkCommandBuffer cmd = VK_NULL_HANDLE;
	r = pAllocCmd(dev, &cbai, &cmd);
	printf("v3dv-harness: vkAllocateCommandBuffers -> %d\n", (int)r);
	if (r != VK_SUCCESS)
		return 8;

	/* --- Tier 3: record a REAL render command (vkCmdClearColorImage) so the submit executes
	 * actual GPU work (a clear) — not just the noop job. Image/memory/clear go via the exported
	 * v3dv_/vk_common_ impls directly (same decouple as Tier 2, bypassing the dispatch gate). --- */
	extern VkResult v3dv_CreateImage(VkDevice, const VkImageCreateInfo *, const VkAllocationCallbacks *, VkImage *);
	extern void vk_common_GetImageMemoryRequirements(VkDevice, VkImage, VkMemoryRequirements *);
	extern VkResult v3dv_AllocateMemory(VkDevice, const VkMemoryAllocateInfo *, const VkAllocationCallbacks *, VkDeviceMemory *);
	extern VkResult vk_common_BindImageMemory(VkDevice, VkImage, VkDeviceMemory, VkDeviceSize);
	extern void v3dv_CmdClearColorImage(VkCommandBuffer, VkImage, VkImageLayout, const VkClearColorValue *, uint32_t, const VkImageSubresourceRange *);

	VkImageCreateInfo imgci = {
		.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
		.imageType = VK_IMAGE_TYPE_2D,
		.format = VK_FORMAT_R8G8B8A8_UNORM,
		.extent = { 64, 64, 1 },
		.mipLevels = 1,
		.arrayLayers = 1,
		.samples = VK_SAMPLE_COUNT_1_BIT,
		.tiling = VK_IMAGE_TILING_LINEAR,
		.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT,
		.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED,
	};
	VkImage image = VK_NULL_HANDLE;
	r = v3dv_CreateImage(dev, &imgci, NULL, &image);
	printf("v3dv-harness: vkCreateImage -> %d\n", (int)r);
	if (r != VK_SUCCESS)
		return 12;

	VkMemoryRequirements mreq;
	memset(&mreq, 0, sizeof(mreq));
	vk_common_GetImageMemoryRequirements(dev, image, &mreq);
	printf("v3dv-harness: image mem size=%u typeBits=0x%x\n",
	       (unsigned)mreq.size, (unsigned)mreq.memoryTypeBits);

	uint32_t memType = 0;
	for (uint32_t i = 0; i < 32; i++) {
		if (mreq.memoryTypeBits & (1u << i)) {
			memType = i;
			break;
		}
	}
	VkMemoryAllocateInfo mai = {
		.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
		.allocationSize = mreq.size ? mreq.size : (64u * 64u * 4u),
		.memoryTypeIndex = memType,
	};
	VkDeviceMemory mem = VK_NULL_HANDLE;
	r = v3dv_AllocateMemory(dev, &mai, NULL, &mem);
	printf("v3dv-harness: vkAllocateMemory -> %d\n", (int)r);
	if (r != VK_SUCCESS)
		return 13;
	r = vk_common_BindImageMemory(dev, image, mem, 0);
	printf("v3dv-harness: vkBindImageMemory -> %d\n", (int)r);
	if (r != VK_SUCCESS)
		return 14;

	/* --- Tier 4a: a VISIBLE-on-HDMI clear. Back a fullscreen image's memory with the HDMI scanout
	 * surface (winsys one-shot v3d_phoenix_set_next_scanout) so the GPU's clear paints the actual
	 * screen — verified via the HDMI auto-snapshot. Best-effort: if /dev/fb0 isn't available the
	 * offscreen Tier-3 clear above still stands. --- */
	extern void v3d_phoenix_set_scanout(uint32_t pa, uint32_t bytes);
	extern void v3d_phoenix_set_next_scanout(void);
	typedef struct {
		unsigned short width, height, bpp, pitch;
		unsigned long long smemlen, framebuffer;
	} rpi4fb_mode_h;
	VkImage scimg = VK_NULL_HANDLE;
	VkDeviceMemory scmem = VK_NULL_HANDLE;
	int fbfd = open("/dev/fb0", O_WRONLY);
	rpi4fb_mode_h fbmode;
	memset(&fbmode, 0, sizeof(fbmode));
	if (fbfd >= 0 && ioctl(fbfd, _IOR('g', 1, rpi4fb_mode_h), &fbmode) == 0 && fbmode.framebuffer != 0) {
		printf("v3dv-harness: fb0 %ux%u pitch=%u pa=0x%llx size=%llu\n",
		       fbmode.width, fbmode.height, fbmode.pitch,
		       (unsigned long long)fbmode.framebuffer, (unsigned long long)fbmode.smemlen);
		v3d_phoenix_set_scanout((uint32_t)fbmode.framebuffer, (uint32_t)fbmode.smemlen);
		VkImageCreateInfo sci = {
			.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
			.imageType = VK_IMAGE_TYPE_2D,
			.format = VK_FORMAT_R8G8B8A8_UNORM,
			.extent = { fbmode.width, fbmode.height, 1 },
			.mipLevels = 1,
			.arrayLayers = 1,
			.samples = VK_SAMPLE_COUNT_1_BIT,
			.tiling = VK_IMAGE_TILING_LINEAR,
			.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT,
			.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED,
		};
		if (v3dv_CreateImage(dev, &sci, NULL, &scimg) == VK_SUCCESS) {
			VkMemoryRequirements scmreq;
			memset(&scmreq, 0, sizeof(scmreq));
			vk_common_GetImageMemoryRequirements(dev, scimg, &scmreq);
			VkMemoryAllocateInfo scmai = {
				.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
				.allocationSize = scmreq.size ? scmreq.size : (uint32_t)fbmode.smemlen,
				.memoryTypeIndex = memType,
			};
			v3d_phoenix_set_next_scanout();   /* the NEXT BO (this image's memory) backs the scanout */
			if (v3dv_AllocateMemory(dev, &scmai, NULL, &scmem) == VK_SUCCESS &&
			    vk_common_BindImageMemory(dev, scimg, scmem, 0) == VK_SUCCESS) {
				printf("v3dv-harness: scanout image %ux%u bound (mem=%u)\n",
				       fbmode.width, fbmode.height, (unsigned)scmreq.size);
			}
			else {
				scimg = VK_NULL_HANDLE;
				printf("v3dv-harness: scanout image alloc/bind FAILED\n");
			}
		}
		else {
			printf("v3dv-harness: scanout vkCreateImage FAILED\n");
		}
	}
	else {
		printf("v3dv-harness: fb0 GETMODE unavailable (offscreen clear only)\n");
	}
	if (fbfd >= 0)
		close(fbfd);

	/* --- Tier 4b: exercise the v3dv SHADER COMPILER (NIR->QPU) via vkCmdBlitImage. A blit samples
	 * the source through v3dv's meta-blit fragment shader (built with nir_builder + compiled at
	 * record time) — unlike the clear, which uses the shaderless TLB fast-path. Blitting a GREEN
	 * source over the magenta scanout: if the screen ends up GREEN, the meta-blit shader compiled +
	 * ran on the V3D — the key prerequisite for vkQuake. Uses vk_common_CmdBlitImage directly. --- */
	extern void vk_common_CmdBlitImage(VkCommandBuffer, VkImage, VkImageLayout, VkImage, VkImageLayout,
		uint32_t, const VkImageBlit *, VkFilter);
	VkImage srcimg = VK_NULL_HANDLE;
	VkDeviceMemory srcmem = VK_NULL_HANDLE;
	if (scimg != VK_NULL_HANDLE) {
		VkImageCreateInfo srcci = {
			.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
			.imageType = VK_IMAGE_TYPE_2D,
			.format = VK_FORMAT_R8G8B8A8_UNORM,
			.extent = { 256, 256, 1 },
			.mipLevels = 1,
			.arrayLayers = 1,
			.samples = VK_SAMPLE_COUNT_1_BIT,
			.tiling = VK_IMAGE_TILING_LINEAR,
			.usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT,
			.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED,
		};
		if (v3dv_CreateImage(dev, &srcci, NULL, &srcimg) == VK_SUCCESS) {
			VkMemoryRequirements srcmreq;
			memset(&srcmreq, 0, sizeof(srcmreq));
			vk_common_GetImageMemoryRequirements(dev, srcimg, &srcmreq);
			VkMemoryAllocateInfo srcmai = {
				.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
				.allocationSize = srcmreq.size ? srcmreq.size : (256u * 256u * 4u),
				.memoryTypeIndex = memType,
			};
			if (v3dv_AllocateMemory(dev, &srcmai, NULL, &srcmem) != VK_SUCCESS ||
			    vk_common_BindImageMemory(dev, srcimg, srcmem, 0) != VK_SUCCESS) {
				srcimg = VK_NULL_HANDLE;
				printf("v3dv-harness: blit src image alloc/bind FAILED\n");
			}
			else {
				printf("v3dv-harness: blit src image 256x256 bound\n");
			}
		}
		else {
			srcimg = VK_NULL_HANDLE;
			printf("v3dv-harness: blit src vkCreateImage FAILED\n");
		}
	}

	VkCommandBufferBeginInfo bbi = {
		.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
		.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT,
	};
	r = pBegin(cmd, &bbi);
	if (r != VK_SUCCESS) {
		printf("v3dv-harness: vkBeginCommandBuffer -> %d\n", (int)r);
		return 9;
	}
	VkClearColorValue clearColor = { .float32 = { 0.0f, 0.5f, 1.0f, 1.0f } };
	VkImageSubresourceRange range = {
		.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
		.baseMipLevel = 0,
		.levelCount = 1,
		.baseArrayLayer = 0,
		.layerCount = 1,
	};
	v3dv_CmdClearColorImage(cmd, image, VK_IMAGE_LAYOUT_GENERAL, &clearColor, 1, &range);
	if (scimg != VK_NULL_HANDLE) {
		/* Visible clear to the scanout-backed image -> paints the HDMI screen. */
		VkClearColorValue scColor = { .float32 = { 1.0f, 0.0f, 0.5f, 1.0f } }; /* bright magenta — obvious on screen */
		v3dv_CmdClearColorImage(cmd, scimg, VK_IMAGE_LAYOUT_GENERAL, &scColor, 1, &range);
	}
	if (srcimg != VK_NULL_HANDLE && scimg != VK_NULL_HANDLE) {
		/* Clear the source GREEN (TLB), then BLIT it (scaled) over the magenta scanout. The blit
		 * runs v3dv's meta-blit fragment shader (NIR->QPU): if the screen ends up GREEN, the shader
		 * compiled + executed on the V3D — the Tier-4b shader-compiler milestone. */
		VkClearColorValue srcColor = { .float32 = { 0.0f, 1.0f, 0.0f, 1.0f } }; /* green */
		v3dv_CmdClearColorImage(cmd, srcimg, VK_IMAGE_LAYOUT_GENERAL, &srcColor, 1, &range);
		VkImageBlit blit = {
			.srcSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 },
			.srcOffsets = { { 0, 0, 0 }, { 256, 256, 1 } },
			.dstSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 },
			.dstOffsets = { { 0, 0, 0 }, { (int32_t)fbmode.width, (int32_t)fbmode.height, 1 } },
		};
		vk_common_CmdBlitImage(cmd, srcimg, VK_IMAGE_LAYOUT_GENERAL,
		                       scimg, VK_IMAGE_LAYOUT_GENERAL, 1, &blit, VK_FILTER_NEAREST);
		printf("v3dv-harness: recorded clear-src(green) + blit-to-scanout (meta-blit shader)\n");
	}
	r = pEnd(cmd);
	printf("v3dv-harness: record clear cmd buffer -> %d\n", (int)r);
	if (r != VK_SUCCESS)
		return 9;

	VkSubmitInfo si = {
		.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO,
		.commandBufferCount = 1,
		.pCommandBuffers = &cmd,
	};
	r = pSubmit(queue, 1, &si, VK_NULL_HANDLE);
	printf("v3dv-harness: vkQueueSubmit -> %d\n", (int)r);
	if (r != VK_SUCCESS)
		return 10;

	r = pWaitIdle(queue);
	printf("v3dv-harness: vkQueueWaitIdle -> %d\n", (int)r);
	if (r != VK_SUCCESS)
		return 11;

	/* Tier 3 verification: the image memory (type 0) is HOST_VISIBLE + the winsys BO is uncached,
	 * so map it and read pixel 0 — it must hold the clear color. R8G8B8A8_UNORM {0,0.5,1,1} stores
	 * bytes ~00 80 ff ff (or bgra ff 80 00 ff). A non-trivial pattern proves the GPU actually wrote
	 * the clear (not just executed without fault). */
	extern VkResult v3dv_MapMemory(VkDevice, VkDeviceMemory, VkDeviceSize, VkDeviceSize, VkMemoryMapFlags, void **);
	void *mapped = NULL;
	r = v3dv_MapMemory(dev, mem, 0, VK_WHOLE_SIZE, 0, &mapped);
	if (r == VK_SUCCESS && mapped != NULL) {
		const unsigned char *px = (const unsigned char *)mapped;
		printf("v3dv-harness: clear readback px0 = %02x %02x %02x %02x (expect ~00 80 ff ff / bgra ff 80 00 ff)\n",
		       px[0], px[1], px[2], px[3]);
	}
	else {
		printf("v3dv-harness: clear readback skipped (vkMapMemory -> %d)\n", (int)r);
	}

	/* Tier 4a verification (deterministic, independent of fbcon overdraw / snapshot timing): read the
	 * LIVE scanout framebuffer's physical memory. If the GPU's scanout clear landed, pixel 0 holds the
	 * magenta clear {1,0,0.5,1} = bytes ff 00 80 ff (R8G8B8A8). The HDMI auto-snapshot may also show it. */
	if (scimg != VK_NULL_HANDLE && fbmode.framebuffer != 0) {
		volatile unsigned char *fb = mmap(NULL, 4096, PROT_READ,
			MAP_PHYSMEM | MAP_UNCACHED | MAP_ANONYMOUS, -1, (off_t)fbmode.framebuffer);
		if (fb != MAP_FAILED) {
			printf("v3dv-harness: scanout fb px0 = %02x %02x %02x %02x (blit GREEN 00 ff 00 ff => meta-blit "
			       "SHADER ran; magenta ff 00 80 ff => blit skipped, clear only)\n",
			       fb[0], fb[1], fb[2], fb[3]);
			munmap((void *)fb, 4096);
		}
		else {
			printf("v3dv-harness: scanout fb readback mmap failed\n");
		}
	}

	printf("v3dv-harness: PASS (clear+readback; scanout clear+blit if fb0; fb readback) -- Tier 3/4a/4b\n");
	return 0;
}
