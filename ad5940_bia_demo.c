/*
 * AD5940 BIA User-Space Demo — Daemon mode with command socket
 *
 * Built against libiio v1.0 API (stream model).
 *
 * Architecture:
 *   ┌──────────┐   cmd DGRAM   ┌────────────────────┐   data DGRAM   ┌─────────┐
 *   │  Qt GUI  │ ◄────────────►│  ad5940_bia_demo    │◄───────────────│ AD5940  │
 *   │          │  S/T/?/Q      │  (this program)     │  IIO stream    │ driver  │
 *   └──────────┘               └────────────────────┘                 └─────────┘
 *
 * Commands (single-byte datagrams on /tmp/bia_cmd.sock):
 *   'S' — START acquisition (enable IIO buffer, begin streaming)
 *   'T' — STOP  acquisition (disable IIO buffer)
 *   '?' — STATUS query (reply: sweep_points as uint32 LE)
 *   'Q' — QUIT daemon (cleanup and exit)
 *
 * Data output (DGRAM on /tmp/bia_sample.sock):
 *   Before first sample after START: bia_meta_t (sweep_points)
 *   Each measurement cycle:         bia_sample_t (impedance data)
 *
 * Cross-compile for RK3568 (aarch64):
 *   TOOLCHAIN=path/to/aarch64-linux-gnu
 *   LIBIIO=../libiio/install
 *   ${TOOLCHAIN}-gcc -o ad5940_bia_demo ad5940_bia_demo.c \
 *       -I${LIBIIO}/include/iio \
 *       -L${LIBIIO}/lib \
 *       -liio -lm -lpthread -lrt -static
 *
 * Usage:
 *   ./ad5940_bia_demo     # daemon mode: listen on cmd sock, wait for START
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/select.h>
#include <iio.h>

/* ------------------------------------------------------------------ */
/*  Constants                                                         */
/* ------------------------------------------------------------------ */

/* Each measurement cycle produces 4 FIFO words + 1 frequency + 2 RTIA */
#define NUM_DFT_CHANNELS	4
#define NUM_CHANNELS		7

/* Socket paths */
#define BIA_DATA_SOCK_PATH	"/tmp/bia_sample.sock"
#define BIA_CMD_SOCK_PATH	"/tmp/bia_cmd.sock"

/* Command bytes (single-char datagrams) */
#define CMD_START		'S'
#define CMD_STOP		'T'
#define CMD_STATUS		'?'
#define CMD_QUIT		'Q'

/* Magic number for meta-info packet (distinguishes from bia_sample_t) */
#define BIA_META_MAGIC		0xB1A00000u

/* Module parameter sysfs path for sweep_points */
#define SYSFS_SWEEP_POINTS	"/sys/module/ad5940/parameters/sweep_points"

/* ------------------------------------------------------------------ */
/*  Protocol structures                                                */
/* ------------------------------------------------------------------ */

typedef struct {
	float magnitude;		/* |Z| in Ohms */
	float phase;			/* angle(Z) in degrees */
	float resistance;		/* Real part R in Ohms */
	float reactance;		/* Imaginary part X in Ohms */
	uint32_t freq_hz;		/* Excitation frequency in Hz */
	int32_t curr_real;		/* Raw DFT: current real (18-bit signed) */
	int32_t curr_imag;		/* Raw DFT: current imag */
	int32_t volt_real;		/* Raw DFT: voltage real */
	int32_t volt_imag;		/* Raw DFT: voltage imag */
} bia_sample_t;

/*
 * Meta-info packet sent before first data sample after each START.
 * Sent over the same data socket; distinguished by magic != valid freq.
 */
typedef struct {
	uint32_t magic;			/* = BIA_META_MAGIC */
	uint32_t sweep_points;		/* Number of frequency points */
	uint32_t sweep_type;		/* 0=linear, 1=log, 2=custom */
} bia_meta_t;

/* ------------------------------------------------------------------ */
/*  DFT data parsing helpers                                          */
/* ------------------------------------------------------------------ */

static inline int32_t sign_extend_18bit(uint32_t raw)
{
	raw &= 0x3FFFF;
	if (raw & (1U << 17))
		raw |= 0xFFFC0000;
	return (int32_t)raw;
}

/* ------------------------------------------------------------------ */
/*  Impedance calculation                                             */
/* ------------------------------------------------------------------ */

static void compute_impedance(int32_t cr, int32_t ci,
			      int32_t vr, int32_t vi,
			      float rtia_mag_ohm, float rtia_phase_deg,
			      uint32_t freq_hz, bia_sample_t *out)
{
	float volt_mag   = sqrtf((float)vr * vr + (float)vi * vi);
	float volt_phase = atan2f(-vi, vr);
	float curr_mag   = sqrtf((float)cr * cr + (float)ci * ci);
	float curr_phase = atan2f(-ci, cr);

	if (curr_mag < 1e-6f) {
		memset(out, 0, sizeof(*out));
		out->freq_hz = freq_hz;
		return;
	}

	float z_mag   = volt_mag / curr_mag * rtia_mag_ohm;
	float z_phase = volt_phase - curr_phase
		      + rtia_phase_deg * (float)M_PI / 180.0f;

	z_phase = atan2f(sinf(z_phase), cosf(z_phase));

	out->magnitude   = z_mag;
	out->phase       = z_phase * 180.0f / (float)M_PI;
	out->resistance  = z_mag * cosf(z_phase);
	out->reactance   = z_mag * sinf(z_phase);
	out->freq_hz     = freq_hz;
	out->curr_real   = cr;
	out->curr_imag   = ci;
	out->volt_real   = vr;
	out->volt_imag   = vi;
}

/* ------------------------------------------------------------------ */
/*  Global state (shared between threads)                              */
/* ------------------------------------------------------------------ */

static volatile sig_atomic_t g_keep_running = 1;
static volatile int g_acquiring = 0;		/* 1 while actively acquiring */
static volatile int g_sweep_points = 0;	/* read from sysfs at START */
static volatile int g_sweep_type = 0;		/* read from sysfs at START */

static pthread_mutex_t g_state_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_acquire_cond = PTHREAD_COND_INITIALIZER;

static void sigint_handler(int sig)
{
	(void)sig;
	g_keep_running = 0;
	pthread_cond_signal(&g_acquire_cond);
}

/* ------------------------------------------------------------------ */
/*  Sysfs helpers                                                      */
/* ------------------------------------------------------------------ */

static int read_sysfs_int(const char *path)
{
	FILE *f = fopen(path, "r");
	if (!f) return -1;
	int val;
	if (fscanf(f, "%d", &val) != 1)
		val = -1;
	fclose(f);
	return val;
}

static void read_sweep_params(void)
{
	g_sweep_points = read_sysfs_int(SYSFS_SWEEP_POINTS);
	if (g_sweep_points <= 0)
		g_sweep_points = 12;	/* fallback default (custom table size) */

	/* Also try sweep_type */
	int st = read_sysfs_int("/sys/module/ad5940/parameters/sweep_type");
	g_sweep_type = (st >= 0 && st <= 2) ? st : 0;

	printf("[demo] sweep_points=%d, sweep_type=%d\n",
	       g_sweep_points, g_sweep_type);
}

/* ------------------------------------------------------------------ */
/*  Ring buffer for data → communication thread                       */
/* ------------------------------------------------------------------ */

#define RING_SIZE	16

typedef struct {
	bia_sample_t	buf[RING_SIZE];
	int		head, tail, count;
	pthread_mutex_t	mtx;
	pthread_cond_t	cond;
} ringbuf_t;

static ringbuf_t g_ring = {
	.mtx = PTHREAD_MUTEX_INITIALIZER,
	.cond = PTHREAD_COND_INITIALIZER,
};

static void ring_push(ringbuf_t *r, const bia_sample_t *s)
{
	pthread_mutex_lock(&r->mtx);
	if (r->count == RING_SIZE) {
		r->tail = (r->tail + 1) % RING_SIZE;
		r->count--;
	}
	r->buf[r->head] = *s;
	r->head = (r->head + 1) % RING_SIZE;
	r->count++;
	pthread_cond_signal(&r->cond);
	pthread_mutex_unlock(&r->mtx);
}

static int ring_pop(ringbuf_t *r, bia_sample_t *s)
{
	pthread_mutex_lock(&r->mtx);
	while (r->count == 0 && g_keep_running)
		pthread_cond_wait(&r->cond, &r->mtx);
	if (r->count == 0) {
		pthread_mutex_unlock(&r->mtx);
		return -1;
	}
	*s = r->buf[r->tail];
	r->tail = (r->tail + 1) % RING_SIZE;
	r->count--;
	pthread_mutex_unlock(&r->mtx);
	return 0;
}

/* ------------------------------------------------------------------ */
/*  Communication thread: sends samples/meta over data socket          */
/* ------------------------------------------------------------------ */

static void *comm_thread_fn(void *arg)
{
	(void)arg;
	int fd;
	struct sockaddr_un addr;
	int seq = 0;
	bool meta_sent = false;

	fd = socket(AF_UNIX, SOCK_DGRAM, 0);
	if (fd < 0) {
		fprintf(stderr, "[comm] socket(): %s\n", strerror(errno));
		return NULL;
	}

	memset(&addr, 0, sizeof(addr));
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, BIA_DATA_SOCK_PATH, sizeof(addr.sun_path) - 1);

	printf("[comm] Sending data to %s\n", BIA_DATA_SOCK_PATH);

	while (g_keep_running) {
		/* Try to pop a regular sample */
		bia_sample_t sample;
		if (ring_pop(&g_ring, &sample) < 0)
			break;

		/*
		 * Send meta-info packet before the first sample of each round.
		 * This tells the Qt frontend how many sweep_points to expect,
		 * so it can auto-stop after one complete sweep and show progress.
		 */
		if (!meta_sent && g_acquiring) {
			bia_meta_t meta = {
				.magic = BIA_META_MAGIC,
				.sweep_points = (uint32_t)g_sweep_points,
				.sweep_type = (uint32_t)g_sweep_type,
			};
			ssize_t mn = sendto(fd, &meta, sizeof(meta), 0,
					    (struct sockaddr *)&addr,
					    sizeof(addr));
			if ((size_t)mn == sizeof(meta))
				printf("[comm] META sent: %u points, type %u\n",
				       meta.sweep_points, meta.sweep_type);
			else
				fprintf(stderr, "[comm] META send failed: %zd\n",
					mn);
			meta_sent = true;
		}

		/* Reset meta_sent when acquisition stops */
		if (!g_acquiring)
			meta_sent = false;

		seq++;
		ssize_t n = sendto(fd, &sample, sizeof(sample), 0,
				   (struct sockaddr *)&addr, sizeof(addr));
		if ((size_t)n != sizeof(sample))
			fprintf(stderr, "[comm] sendto partial: %zd/%zu\n",
				n, sizeof(sample));
	}

	close(fd);
	printf("[comm] Exiting (%d datagrams sent)\n", seq);
	return NULL;
}

/* ------------------------------------------------------------------ */
/*  Acquisition thread: reads IIO stream, computes impedance            */
/* ------------------------------------------------------------------ */

typedef struct {
	struct iio_context	*ctx;
	struct iio_device	*dev;
	struct iio_channel	*ch[NUM_CHANNELS];
	struct iio_buffer	*buf;
	struct iio_channels_mask *mask;
	struct iio_stream	*stream;
} iio_state_t;

/*
 * enable_buffer — Enable/disable the IIO triggered buffer.
 * Returns 0 on success.
 */
static int enable_iio_buffer(struct iio_device *dev, bool en)
{
	const char *val = en ? "1" : "0";
	char path[256];

	snprintf(path, sizeof(path),
		 "/sys/bus/iio/devices/%s/buffer/enable",
		 iio_device_get_id(dev));
	FILE *f = fopen(path, "w");
	if (!f) {
		fprintf(stderr, "[demo] Cannot open %s: %s\n",
			path, strerror(errno));
		return -1;
	}
	int ok = (fwrite(val, 1, 1, f) == 1);
	fclose(f);

	printf("[demo] IIO buffer %s (%s)\n",
	       en ? "ENABLED" : "DISABLED", path);
	return ok ? 0 : -1;
}

static void *acq_thread_fn(void *arg)
{
	iio_state_t *s = (iio_state_t *)arg;
	int sample_count = 0;

	while (g_keep_running) {
		/* Wait for START command */
		pthread_mutex_lock(&g_state_mtx);
		while (!g_acquiring && g_keep_running)
			pthread_cond_wait(&g_acquire_cond, &g_state_mtx);
		if (!g_keep_running) {
			pthread_mutex_unlock(&g_state_mtx);
			break;
		}
		pthread_mutex_unlock(&g_state_mtx);

		/*
		 * ---- Create a fresh stream for this acquisition round ----
		 *
		 * iio_stream_cancel() permanently invalidates a stream object;
		 * there is no "uncancel" API. So we must destroy and recreate
		 * the stream for each round. This also ensures the buffer
		 * enable/disable lifecycle is clean.
		 */
		s->stream = iio_buffer_create_stream(s->buf, 4, 1, s->mask);
		if (iio_err(s->stream)) {
			fprintf(stderr, "[demo] Cannot create stream: %s\n",
				strerror(-iio_err(s->stream)));
			pthread_mutex_lock(&g_state_mtx);
			g_acquiring = 0;
			pthread_mutex_unlock(&g_state_mtx);
			continue;
		}

		sample_count = 0;

		/*
		 * ---- Acquisition loop ----
		 *
		 * iio_stream_get_next_block() internally handles buffer/enable
		 * on the first call. We do NOT manually write sysfs enable=1.
		 *
		 * Stop conditions:
		 *   a) User presses STOP → main thread calls
		 *      iio_stream_cancel() + sets g_acquiring=0
		 *   b) Auto-stop: sample_count >= g_sweep_points
		 *      (prevents driver sweep wrap-around from producing
		 *      an extra sample before Qt's STOP arrives)
		 */
		while (g_keep_running && g_acquiring) {
			const struct iio_block *block;

			block = iio_stream_get_next_block(s->stream);
			if (!block || iio_err(block)) {
				if (!g_keep_running || !g_acquiring)
					break;	/* Expected: STOP cancelled */
				fprintf(stderr, "[demo] Stream error: %s\n",
					iio_err(block) ?
					strerror(-iio_err(block)) : "null");
				break;
			}

			/* Extract channel data from this block */
			int32_t raw[NUM_CHANNELS];
			int64_t raw_rtia_mag;
			int32_t raw_rtia_phase;
			unsigned int i;

			for (i = 0; i < NUM_CHANNELS - 2; i++) {
				size_t nr = iio_channel_read(s->ch[i], block,
							     &raw[i],
							     sizeof(int32_t),
							     true);
				if (nr < sizeof(int32_t))
					goto acq_err;
			}
			/* RTIA channels: resistance0=int64, phase0=int32 */
			iio_channel_read(s->ch[5], block,
					 &raw_rtia_mag, sizeof(int64_t), true);
			iio_channel_read(s->ch[6], block,
					 &raw_rtia_phase, sizeof(int32_t),
					 true);

			int32_t cr = sign_extend_18bit((uint32_t)raw[0]);
			int32_t ci = sign_extend_18bit((uint32_t)raw[1]);
			int32_t vr = sign_extend_18bit((uint32_t)raw[2]);
			int32_t vi = sign_extend_18bit((uint32_t)raw[3]);

			float rtm = (float)raw_rtia_mag / 1000.0f;
			float rtp = (float)raw_rtia_phase / 1000.0f;
			uint32_t freq = (uint32_t)raw[4];

			bia_sample_t sample;
			compute_impedance(cr, ci, vr, vi, rtm, rtp, freq, &sample);

			ring_push(&g_ring, &sample);
			sample_count++;

			/*
			 * ---- Auto-stop: one sweep round complete ----
			 * The driver's sweep_index wraps around to 0 after
			 * sweep_points. Without this check, we'd keep reading
			 * the first frequency point again before Qt's STOP
			 * command arrives through the socket.
			 */
			if (g_sweep_points > 0 &&
			    sample_count >= g_sweep_points) {
				printf("[demo] Sweep complete: %d/%d samples\n",
				       sample_count, g_sweep_points);
				break;
			}
		}

	acq_err:
		/* ---- Teardown this round ---- */
		/* Cancel any pending block read, then destroy stream */
		iio_stream_cancel(s->stream);
		iio_stream_destroy(s->stream);
		s->stream = NULL;

		/* Stop the hardware via sysfs (tells driver to stop WUPT) */
		enable_iio_buffer(s->dev, false);

		pthread_mutex_lock(&g_state_mtx);
		g_acquiring = 0;
		pthread_mutex_unlock(&g_state_mtx);

		printf("[demo] Acquisition stopped, %d samples\n", sample_count);

		/* Small delay before re-entering wait state */
		usleep(100000);
	}

	return NULL;
}

/* ------------------------------------------------------------------ */
/*  Main: command listener loop                                       */
/* ------------------------------------------------------------------ */

int main(int argc, char *argv[])
{
	iio_state_t s = {0};
	int ret = 0;
	int cmd_fd = -1;
	pthread_t acq_tid, comm_tid;

	(void)argc;
	(void)argv;

	signal(SIGINT, sigint_handler);
	signal(SIGTERM, sigint_handler);

	/* ---- Create IIO context (local, persistent) ---- */
	s.ctx = iio_create_context(NULL, "local:");
	if (iio_err(s.ctx)) {
		fprintf(stderr, "ERROR: Cannot create IIO context: %s\n",
			strerror(-iio_err(s.ctx)));
		return 1;
	}
	iio_context_set_timeout(s.ctx, 0);	/* blocking, no timeout */

	/* ---- Find AD5940 device ---- */
	s.dev = iio_context_find_device(s.ctx, "ad5940");
	if (!s.dev) {
		fprintf(stderr, "ERROR: Device 'ad5940' not found. "
			"Is module loaded?\n");
		ret = 1;
		goto cleanup;
	}

	printf("AD5940 BIA Daemon v2 (libiio v%u.%u)\n",
	       iio_context_get_version_major(s.ctx),
	       iio_context_get_version_minor(s.ctx));

	/* ---- Get channels ---- */
	s.ch[0] = iio_device_find_channel(s.dev, "current0", false);
	s.ch[1] = iio_device_find_channel(s.dev, "current1", false);
	s.ch[2] = iio_device_find_channel(s.dev, "voltage0", false);
	s.ch[3] = iio_device_find_channel(s.dev, "voltage1", false);
	s.ch[4] = iio_device_find_channel(s.dev, "altvoltage0", false);
	s.ch[5] = iio_device_find_channel(s.dev, "resistance0", false);
	s.ch[6] = iio_device_find_channel(s.dev, "phase0", false);
	for (int i = 0; i < NUM_CHANNELS; i++) {
		if (!s.ch[i]) {
			fprintf(stderr, "ERROR: Channel %d not found\n", i);
			ret = 1;
			goto cleanup;
		}
	}

	/* ---- Setup IIO buffer (persistent; stream is created per-round) ---- */
	s.buf = iio_device_get_buffer(s.dev, 0);
	if (iio_err(s.buf)) {
		fprintf(stderr, "ERROR: Cannot get IIO buffer: %s\n",
			strerror(-iio_err(s.buf)));
		ret = 1;
		goto cleanup;
	}

	s.mask = iio_create_channels_mask(
		iio_device_get_channels_count(s.dev));
	if (!s.mask) {
		fprintf(stderr, "ERROR: Cannot create channels mask\n");
		ret = 1;
		goto cleanup;
	}
	for (int i = 0; i < NUM_CHANNELS; i++)
		iio_channel_enable(s.ch[i], s.mask);

	/* Stream will be created/destroyed per acquisition round by
	 * acq_thread_fn, because iio_stream_cancel() permanently
	 * invalidates a stream object. */

	/* ---- Start communication thread ---- */
	ret = pthread_create(&comm_tid, NULL, comm_thread_fn, NULL);
	if (ret) {
		fprintf(stderr, "ERROR: Cannot create comm thread: %s\n",
			strerror(ret));
		ret = 1;
		goto cleanup;
	}

	/* ---- Start acquisition thread (will wait for START cmd) ---- */
	ret = pthread_create(&acq_tid, NULL, acq_thread_fn, &s);
	if (ret) {
		fprintf(stderr, "ERROR: Cannot create acq thread: %s\n",
			strerror(ret));
		g_keep_running = 0;
		pthread_cond_signal(&g_acquire_cond);
		pthread_join(comm_tid, NULL);
		ret = 1;
		goto cleanup;
	}

	/* ---- Bind command socket ---- */
	cmd_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
	if (cmd_fd < 0) {
		fprintf(stderr, "[cmd] socket(): %s\n", strerror(errno));
		goto cleanup_threads;
	}

	unlink(BIA_CMD_SOCK_PATH);
	struct sockaddr_un cmd_addr;
	memset(&cmd_addr, 0, sizeof(cmd_addr));
	cmd_addr.sun_family = AF_UNIX;
	strncpy(cmd_addr.sun_path, BIA_CMD_SOCK_PATH, sizeof(cmd_addr.sun_path) - 1);

	if (bind(cmd_fd, (struct sockaddr *)&cmd_addr, sizeof(cmd_addr)) < 0) {
		fprintf(stderr, "[cmd] bind(%s): %s\n",
			BIA_CMD_SOCK_PATH, strerror(errno));
		close(cmd_fd);
		cmd_fd = -1;
		goto cleanup_threads;
	}

	/* Set receive buffer size */
	int rcvbuf = 4096;
	setsockopt(cmd_fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

	printf("[demo] Daemon ready. Listening on %s\n", BIA_CMD_SOCK_PATH);
	printf("[demo] Commands: S=START T=STOP ?=STATUS Q=QUIT\n");

	/* ---- Command loop ---- */
	while (g_keep_running) {
		char cmd;
		ssize_t n = recvfrom(cmd_fd, &cmd, 1, 0, NULL, NULL);
		if (n < 0) {
			if (errno == EINTR || errno == EAGAIN)
				continue;
			if (errno == ECONNREFUSED || errno == ENOENT)
				continue;	/* nobody bound yet, normal */
			fprintf(stderr, "[cmd] recvfrom: %s\n", strerror(errno));
			continue;
		}
		if (n == 0)
			continue;

		switch (cmd) {
		case CMD_START:
			read_sweep_params();
			pthread_mutex_lock(&g_state_mtx);
			g_acquiring = 1;
			pthread_cond_signal(&g_acquire_cond);
			pthread_mutex_unlock(&g_state_mtx);
			printf("[cmd] → START (points=%d)\n", g_sweep_points);
			break;

		case CMD_STOP:
			/*
			 * Cancel the stream to unblock iio_stream_get_next_block().
			 * Stream may already be NULL if acq thread auto-stopped
			 * after sweep_points (safe to call with NULL check).
			 */
			if (s.stream)
				iio_stream_cancel(s.stream);
			pthread_mutex_lock(&g_state_mtx);
			g_acquiring = 0;
			pthread_mutex_unlock(&g_state_mtx);
			printf("[cmd] → STOP\n");
			break;

		case CMD_STATUS: {
			/* Reply: send sweep_points as uint32 LE */
			uint32_t sp = (uint32_t)(g_acquiring ?
					g_sweep_points : 0);
			struct sockaddr_un reply_addr;
			socklen_t alen = sizeof(reply_addr);
			/* Peek sender address by doing a throwaway recv?
			 * Simpler: just send back to known Qt data socket */
			sendto(cmd_fd, &sp, sizeof(sp), 0,
			       (struct sockaddr *)&cmd_addr, sizeof(cmd_addr));
			break;
		}

		case CMD_QUIT:
			printf("[cmd] → QUIT\n");
			g_keep_running = 0;
			pthread_mutex_lock(&g_state_mtx);
			g_acquiring = 0;
			pthread_cond_signal(&g_acquire_cond);
			pthread_mutex_unlock(&g_state_mtx);
			break;

		default:
			printf("[cmd] Unknown: 0x%02x ('%c')\n",
			       (unsigned char)cmd, cmd);
			break;
		}
	}

cleanup_threads:
	/* Signal all threads to exit */
	g_keep_running = 0;
	pthread_cond_signal(&g_acquire_cond);
	/* Wake ring buffer so comm thread exits */
	pthread_cond_signal(&g_ring.cond);

	if (acq_tid)
		pthread_join(acq_tid, NULL);
	pthread_join(comm_tid, NULL);

	if (cmd_fd >= 0)
		close(cmd_fd);
	unlink(BIA_CMD_SOCK_PATH);

cleanup:
	/* Stream is created/destroyed per round in acq_thread_fn;
	 * s.stream should be NULL here unless we exited early. */
	if (s.stream)
		iio_stream_destroy(s.stream);
	if (s.mask)
		iio_channels_mask_destroy(s.mask);
	/* s.buf and s.channels are owned by s.ctx */
	if (s.ctx)
		iio_context_destroy(s.ctx);

	printf("[demo] Exited.\n");
	return ret;
}
