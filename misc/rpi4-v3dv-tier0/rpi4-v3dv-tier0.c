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

	if (pProps) {
		VkPhysicalDeviceProperties props;
		memset(&props, 0, sizeof(props));
		pProps(phys, &props);
		printf("v3dv-harness: device name='%s' apiVersion=0x%x\n",
		       props.deviceName, props.apiVersion);
	}

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

	printf("v3dv-harness: PASS (instance+phys+device created)\n");
	return 0;
}
