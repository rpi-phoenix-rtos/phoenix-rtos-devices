/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) boot system-info banner (rpi4-sysinfo)
 *
 * Prints a one-screen branded summary at boot: build stamp, uptime, a hardware-RNG
 * entropy sample (via getentropy), and an inventory of the Pi 4 device nodes. It only
 * uses bounded, non-blocking calls (stat / getentropy / clock_gettime), so it cannot
 * stall the boot. A "the system is alive" touch that also doubles as a quick device
 * presence check.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 */
#include <stdio.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/random.h>


static void sysinfo_inventory(void)
{
	static const char *const nodes[] = {
		"/dev/console", "/dev/null", "/dev/zero", "/dev/urandom",
		"/dev/hwrng", "/dev/thermal", "/dev/throttled", "/dev/gpio",
		"/dev/fb0", "/dev/audio0", "/dev/kbd0", "/dev/mouse0",
	};
	size_t i;
	struct stat st;

	printf("rpi4-sysinfo: devices:");
	for (i = 0; i < sizeof(nodes) / sizeof(nodes[0]); i++) {
		printf(" %s%s", nodes[i] + 5 /* skip "/dev/" */, (stat(nodes[i], &st) == 0) ? "+" : "-");
	}
	printf("  (+present -absent)\n");
}


int main(int argc, char **argv)
{
	unsigned char ent[8];
	struct timespec ts;
	int i;

	(void)argc;
	(void)argv;

	printf("================================================================\n");
	printf(" Phoenix-RTOS  ::  Raspberry Pi 4 (BCM2711, Cortex-A72)\n");
	printf(" rpi4-sysinfo  built %s %s\n", __DATE__, __TIME__);
	printf("================================================================\n");

	if (clock_gettime(CLOCK_MONOTONIC, &ts) == 0) {
		printf("rpi4-sysinfo: uptime %ld.%03ld s\n", (long)ts.tv_sec, (long)(ts.tv_nsec / 1000000L));
	}

	/* getentropy draws from /dev/urandom -> the BCM2711 hardware RNG. */
	if (getentropy(ent, sizeof(ent)) == 0) {
		printf("rpi4-sysinfo: hw entropy");
		for (i = 0; i < (int)sizeof(ent); i++) {
			printf(" %02x", ent[i]);
		}
		printf("\n");
	}

	sysinfo_inventory();
	printf("================================================================\n");

	return 0;
}
