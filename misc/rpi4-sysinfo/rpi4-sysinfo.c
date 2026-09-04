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
#include <string.h>
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


/*
 * Print WHICH COMMIT of every Phoenix repo went into this build.
 *
 * Owner request 2026-09-05: a UART log should say what the system IS, not only
 * what it did -- otherwise triage starts by guessing which tree produced the
 * binaries. scripts/gen-build-versions.sh writes the file at build time, one
 * line per repo: "<repo> <short-sha>[+dirty] <date>". The dirty marker is the
 * important half: a build from a modified tree is exactly where a bare commit
 * id misleads.
 *
 * Bounded on purpose -- at most BUILDVER_MAX_LINES lines of at most
 * BUILDVER_MAX_COLS characters -- so a truncated or corrupt file cannot flood
 * the boot console, and a missing file just says so (the netboot RAM root has
 * no /etc, and that is not a failure).
 */
#define BUILDVER_PATH      "/etc/build-versions"
#define BUILDVER_MAX_LINES 32
#define BUILDVER_MAX_COLS  110

static void sysinfo_buildVersions(void)
{
	FILE *f;
	char line[BUILDVER_MAX_COLS + 2];
	int printed = 0;

	f = fopen(BUILDVER_PATH, "r");
	if (f == NULL) {
		printf("rpi4-sysinfo: no %s (component commit ids unavailable)\n", BUILDVER_PATH);
		return;
	}

	printf("rpi4-sysinfo: build components (%s):\n", BUILDVER_PATH);
	while ((printed < BUILDVER_MAX_LINES) && (fgets(line, sizeof(line), f) != NULL)) {
		size_t len = strlen(line);

		while ((len > 0u) && ((line[len - 1u] == '\n') || (line[len - 1u] == '\r'))) {
			line[--len] = '\0';
		}
		if ((len == 0u) || (line[0] == '#')) {
			continue;
		}
		printf("  %s\n", line);
		printed++;
	}
	if (printed == BUILDVER_MAX_LINES) {
		printf("  ... (truncated at %d lines)\n", BUILDVER_MAX_LINES);
	}

	fclose(f);
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
	sysinfo_buildVersions();
	printf("================================================================\n");

	return 0;
}
