/*
 * dummy_Qt.c — Simulates the Qt-side process that receives BIA samples
 * via Unix DGRAM socket and prints them.
 *
 * Must be started BEFORE ad5940_bia_demo so the socket endpoint exists.
 *
 * Cross-compile for RK3568 (aarch64):
 *   aarch64-linux-gnu-gcc -o dummy_Qt dummy_Qt.c -static
 *
 * Usage:
 *   ./dummy_Qt            # run forever until Ctrl-C
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>

/* Must match ad5940_bia_demo.c */
#define BIA_SOCK_PATH	"/tmp/bia_sample.sock"

typedef struct {
	float magnitude;	/* |Z| in Ohms */
	float phase;		/* angle(Z) in degrees */
	float resistance;	/* Real part R in Ohms */
	float reactance;	/* Imaginary part X in Ohms */
	uint32_t freq_hz;	/* Excitation frequency in Hz */
} bia_sample_t;

static volatile sig_atomic_t keep_running = 1;

static void sigint_handler(int sig)
{
	(void)sig;
	keep_running = 0;
}

int main(int argc, char *argv[])
{
	int fd;
	struct sockaddr_un addr;
	bia_sample_t sample;
	int seq = 0;

	(void)argc;
	(void)argv;

	signal(SIGINT, sigint_handler);
	signal(SIGTERM, sigint_handler);

	/* Create and bind Unix DGRAM socket */
	fd = socket(AF_UNIX, SOCK_DGRAM, 0);
	if (fd < 0) {
		fprintf(stderr, "ERROR: socket(): %s\n", strerror(errno));
		return 1;
	}

	memset(&addr, 0, sizeof(addr));
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, BIA_SOCK_PATH, sizeof(addr.sun_path) - 1);

	/* Remove stale socket file if it exists */
	unlink(BIA_SOCK_PATH);

	if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		fprintf(stderr, "ERROR: bind(): %s\n", strerror(errno));
		close(fd);
		return 1;
	}

	/* Set receive buffer to avoid drops if we process slowly */
	int rcvbuf = 65536;
	setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

	printf("dummy_Qt: Listening on %s (waiting for samples...)\n",
	       BIA_SOCK_PATH);
	printf("%-6s %10s %12s %12s %12s %12s\n",
	       "#", "Freq(Hz)", "|Z|(Ohm)", "Phase(deg)",
	       "R(Ohm)", "X(Ohm)");
	printf("------ ---------- ------------ ------------ ------------ ------------\n");

	while (keep_running) {
		ssize_t n = recvfrom(fd, &sample, sizeof(sample), 0,
				     NULL, NULL);
		if (n < 0) {
			if (errno == EINTR)
				continue;
			fprintf(stderr, "ERROR: recvfrom(): %s\n",
				strerror(errno));
			break;
		}

		if ((size_t)n != sizeof(sample)) {
			fprintf(stderr, "WARNING: truncated datagram %zd/%zu\n",
				n, sizeof(sample));
			continue;
		}

		seq++;
		printf("%-6d %10u %12.2f %12.2f %12.2f %12.2f\n",
		       seq, sample.freq_hz,
		       sample.magnitude, sample.phase,
		       sample.resistance, sample.reactance);
	}

	printf("\ndummy_Qt: Received %d samples, exiting\n", seq);

	close(fd);
	unlink(BIA_SOCK_PATH);

	return 0;
}
