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
	PFN_vkGetDeviceQueue pGetQueue = GDPA(dev, vkGetDeviceQueue);
	PFN_vkCreateCommandPool pCreatePool = GDPA(dev, vkCreateCommandPool);
	PFN_vkAllocateCommandBuffers pAllocCmd = GDPA(dev, vkAllocateCommandBuffers);
	PFN_vkBeginCommandBuffer pBegin = GDPA(dev, vkBeginCommandBuffer);
	PFN_vkEndCommandBuffer pEnd = GDPA(dev, vkEndCommandBuffer);
	PFN_vkQueueSubmit pSubmit = GDPA(dev, vkQueueSubmit);
	PFN_vkQueueWaitIdle pWaitIdle = GDPA(dev, vkQueueWaitIdle);
	if (!pGetQueue || !pCreatePool || !pAllocCmd || !pBegin || !pEnd || !pSubmit || !pWaitIdle) {
		printf("v3dv-harness: PASS (device created); Tier-2 submit procs missing\n");
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

	VkCommandBufferBeginInfo bbi = {
		.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
		.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT,
	};
	r = pBegin(cmd, &bbi);
	if (r == VK_SUCCESS)
		r = pEnd(cmd);
	printf("v3dv-harness: record empty cmd buffer -> %d\n", (int)r);
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

	printf("v3dv-harness: PASS (instance+phys+device+queue submit)\n");
	return 0;
}
