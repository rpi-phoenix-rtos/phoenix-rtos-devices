/*
 * v3d_libdrm_shim.c — libdrm surface impls for the v3d gallium driver on Phoenix.
 *
 * drmIoctl() itself is an inline in xf86drm.h that forwards to phoenix_v3d_ioctl
 * (the winsys). This file provides the rest of the libdrm surface the driver
 * references. Our SUBMIT_CL is SYNCHRONOUS (we block on FLDONE/FRDONE inside the
 * winsys), so fences are always already-signaled: every syncobj op returns success
 * and reports "signaled". BO sharing (PRIME) is not supported (single client).
 *
 * Copyright 2026 Phoenix Systems  %LICENSE%
 */
#include <stddef.h>
#include "xf86drm.h"

/* Synchronous submit => a created fence is immediately satisfiable. */
int drmSyncobjCreate(int fd, uint32_t flags, uint32_t *handle)
{
	(void)fd; (void)flags;
	if (handle)
		*handle = 1;          /* any nonzero handle */
	return 0;
}

int drmSyncobjDestroy(int fd, uint32_t handle)
{
	(void)fd; (void)handle;
	return 0;
}

/* Submit already completed before this is called -> report all signaled. */
int drmSyncobjWait(int fd, uint32_t *handles, unsigned num_handles,
                   int64_t timeout_nsec, unsigned flags, uint32_t *first_signaled)
{
	(void)fd; (void)handles; (void)timeout_nsec; (void)flags;
	if (first_signaled && num_handles)
		*first_signaled = 0;  /* index of the first signaled handle */
	return 0;
}

int drmSyncobjImportSyncFile(int fd, uint32_t handle, int sync_file_fd)
{
	(void)fd; (void)handle; (void)sync_file_fd;
	return 0;
}

int drmSyncobjExportSyncFile(int fd, uint32_t handle, int *sync_file_fd)
{
	(void)fd; (void)handle;
	if (sync_file_fd)
		*sync_file_fd = -1;
	return 0;
}

/* No buffer sharing on Phoenix (single client) -> PRIME unsupported. */
int drmPrimeHandleToFD(int fd, uint32_t handle, uint32_t flags, int *prime_fd)
{
	(void)fd; (void)handle; (void)flags;
	if (prime_fd)
		*prime_fd = -1;
	return -1;
}

int drmPrimeFDToHandle(int fd, int prime_fd, uint32_t *handle)
{
	(void)fd; (void)prime_fd; (void)handle;
	return -1;
}

int drmGetCap(int fd, uint64_t capability, uint64_t *value)
{
	(void)fd; (void)capability;
	if (value)
		*value = 0;
	return 0;
}

char *drmGetRenderDeviceNameFromFd(int fd)
{
	(void)fd;
	return NULL;
}
