/*
 * rpi4-quake-stub.c — placeholder translation unit for the rpi4-quake program.
 *
 * The actual Quakespasm engine (including main(), in pl_phoenix_main.o) lives in
 * /tmp/libquakespasm.a, built by tools/quakespasm-port/build-quakespasm-phoenix.py
 * with the correct per-file flag sets (Quake-side vs Mesa-side). binary.mk needs a
 * LOCAL_SRCS file to produce the program; the linker pulls main() (and transitively
 * the whole engine) from the archive via crt0's reference to main.
 */
int rpi4_quake_stub_marker;
