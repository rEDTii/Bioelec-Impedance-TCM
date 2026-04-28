/*
 * AD5940 BIA User-Space Demo — Read DFT data via IIO and compute impedance
 *
 * Built against libiio v1.0 API (stream model).
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
 *   ./ad5940_bia_demo                    # default: 100 samples
 *   ./ad5940_bia_demo 0                  # run forever until Ctrl-C
 *   ./ad5940_bia_demo 500                # collect 500 samples
 *
 * Prerequisites:
 *   - ad5940 kernel module loaded
 *   - IIO buffer enabled (or use AFE_enable.sh)
 *
 * Data flow:
 *   AD5940 FIFO (4 words per measurement cycle) + 1 frequency word:
 *     word0 = Current DFT Real   (18-bit signed, bits[17:0])
 *     word1 = Current DFT Imag
 *     word2 = Voltage DFT Real
 *     word3 = Voltage DFT Imag
 *     word4 = Excitation Frequency (32-bit unsigned, in Hz)
 *
 *   Impedance calculation (from ADI BodyImpedance.c AppBIAISR()):
 *     |Z| = |V| / |I| * Rtia
 *     angle(Z) = angle(V) - angle(I)
 *   where Rtia is the HSTIA feedback resistor (default 1kOhm for HSTIARTIA_1K).
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
#include <iio.h>

/* ------------------------------------------------------------------ */
/*  BIA parameters — must match kernel driver configuration           */
/* ------------------------------------------------------------------ */

/*
 * Rtia: HSTIA feedback resistor value in Ohms.
 * HstiaRtiaSel = HSTIARTIA_1K → Rtia = 1000 Ohm
 *
 * Note: ADI's AppBIAISR() uses RtiaCurrValue[] from AD5940_HSRtiaCal()
 * which returns the calibrated Rtia (magnitude + phase). For this demo
 * we use the nominal 1kOhm value. For precision measurements, you should
 * run the Rtia calibration procedure and substitute the calibrated value.
 */
#define RTIA_NOMINAL_OHM	1000.0f

/* Each measurement cycle produces 4 FIFO words + 1 frequency + 2 RTIA */
#define NUM_DFT_CHANNELS	4
#define NUM_CHANNELS		7

/* ------------------------------------------------------------------ */
/*  DFT data parsing helpers                                          */
/* ------------------------------------------------------------------ */

/*
 * sign_extend_18bit — Convert 18-bit two's complement to int32_t
 *
 * AD5940 DFT results are 18-bit signed values in bits[17:0] of a
 * 32-bit FIFO word. Bit17 is the sign bit.
 * This matches ADI's AppBIAISR() logic:
 *   pData[i] &= 0x3ffff;
 *   if(pData[i]&(1<<17)) pData[i] |= 0xfffc0000;
 */
static inline int32_t sign_extend_18bit(uint32_t raw)
{
	raw &= 0x3FFFF;
	if (raw & (1U << 17))
		raw |= 0xFFFC0000;
	return (int32_t)raw;
}

/* ------------------------------------------------------------------ */
/*  Impedance calculation (mirrors ADI AppBIAISR)                     */
/* ------------------------------------------------------------------ */

typedef struct {
	float magnitude;	/* |Z| in Ohms */
	float phase;		/* angle(Z) in degrees */
	float resistance;	/* Real part R in Ohms */
	float reactance;	/* Imaginary part X in Ohms */
	uint32_t freq_hz;	/* Excitation frequency in Hz */
} bia_sample_t;

/*
 * compute_impedance — Calculate impedance from DFT voltage and current
 *
 * Formula (from ADI BodyImpedance.c):
 *   VoltMag   = sqrt(Vr^2 + Vi^2)
 *   VoltPhase = atan2(-Vi, Vr)      // Note: ADI negates imaginary
 *   CurrMag   = sqrt(Ir^2 + Ii^2)
 *   CurrPhase = atan2(-Ii, Ir)
 *   |Z|       = VoltMag / CurrMag * RtiaCal.magnitude
 *   angle(Z)  = VoltPhase - CurrPhase + RtiaCal.phase
 *
 * Rtia magnitude is in milliohms, phase in millidegrees (from driver).
 */
static void compute_impedance(int32_t curr_real, int32_t curr_imag,
			      int32_t volt_real, int32_t volt_imag,
			      float rtia_mag_ohm, float rtia_phase_deg,
			      uint32_t freq_hz,
			      bia_sample_t *result)
{
	float vr = (float)volt_real;
	float vi = (float)volt_imag;
	float ir = (float)curr_real;
	float ii = (float)curr_imag;

	float volt_mag   = sqrtf(vr * vr + vi * vi);
	float volt_phase = atan2f(-vi, vr);	/* ADI convention */
	float curr_mag   = sqrtf(ir * ir + ii * ii);
	float curr_phase = atan2f(-ii, ir);

	if (curr_mag < 1e-6f) {
		result->magnitude  = 0.0f;
		result->phase      = 0.0f;
		result->resistance = 0.0f;
		result->reactance  = 0.0f;
		return;
	}

	float z_mag   = volt_mag / curr_mag * rtia_mag_ohm;
	float z_phase = volt_phase - curr_phase
		      + rtia_phase_deg * (float)M_PI / 180.0f;

	float z_phase_deg = z_phase * 180.0f / (float)M_PI;

	result->magnitude  = z_mag;
	result->phase      = z_phase_deg;
	result->resistance = z_mag * cosf(z_phase);
	result->reactance  = z_mag * sinf(z_phase);
	result->freq_hz    = freq_hz;
}

/* ------------------------------------------------------------------ */
/*  Signal handling                                                   */
/* ------------------------------------------------------------------ */

static volatile sig_atomic_t keep_running = 1;

static void sigint_handler(int sig)
{
	(void)sig;
	keep_running = 0;
}

/* ------------------------------------------------------------------ */
/*  Ring buffer + communication thread                                */
/* ------------------------------------------------------------------ */

#define RING_BUF_SIZE		16	/* power of 2 for fast modulo */

/* Unix DGRAM socket path — must match dummy_Qt */
#define BIA_SOCK_PATH		"/tmp/bia_sample.sock"

typedef struct {
	bia_sample_t	buf[RING_BUF_SIZE];
	int		head;		/* next write position */
	int		tail;		/* next read position */
	int		count;		/* number of occupied slots */
	pthread_mutex_t	mutex;
	pthread_cond_t	cond;
} ring_buffer_t;

static ring_buffer_t g_ring = {
	.mutex  = PTHREAD_MUTEX_INITIALIZER,
	.cond   = PTHREAD_COND_INITIALIZER,
};

static void ring_push(ring_buffer_t *rb, const bia_sample_t *sample)
{
	pthread_mutex_lock(&rb->mutex);

	/* Overwrite oldest if full (consumer too slow) */
	if (rb->count == RING_BUF_SIZE) {
		rb->tail = (rb->tail + 1) % RING_BUF_SIZE;
		rb->count--;
	}

	rb->buf[rb->head] = *sample;
	rb->head = (rb->head + 1) % RING_BUF_SIZE;
	rb->count++;

	pthread_cond_signal(&rb->cond);
	pthread_mutex_unlock(&rb->mutex);
}

static int ring_pop(ring_buffer_t *rb, bia_sample_t *sample)
{
	pthread_mutex_lock(&rb->mutex);

	while (rb->count == 0 && keep_running)
		pthread_cond_wait(&rb->cond, &rb->mutex);

	if (rb->count == 0) {
		pthread_mutex_unlock(&rb->mutex);
		return -1;	/* shutdown, no data */
	}

	*sample = rb->buf[rb->tail];
	rb->tail = (rb->tail + 1) % RING_BUF_SIZE;
	rb->count--;

	pthread_mutex_unlock(&rb->mutex);
	return 0;
}

/*
 * comm_thread_fn — Communication thread: pops samples and sends them
 * over a Unix DGRAM socket to the Qt process (or dummy_Qt).
 */
static void *comm_thread_fn(void *arg)
{
	bia_sample_t sample;
	int seq = 0;
	int fd = -1;
	struct sockaddr_un addr;

	(void)arg;

	/* Create Unix DGRAM socket */
	fd = socket(AF_UNIX, SOCK_DGRAM, 0);
	if (fd < 0) {
		fprintf(stderr, "[comm] ERROR: socket(): %s\n", strerror(errno));
		return NULL;
	}

	memset(&addr, 0, sizeof(addr));
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, BIA_SOCK_PATH, sizeof(addr.sun_path) - 1);

	printf("[comm] Communication thread started (sending to %s)\n",
	       BIA_SOCK_PATH);

	while (keep_running) {
		if (ring_pop(&g_ring, &sample) < 0)
			break;

		seq++;

		/* Send raw bia_sample_t as one datagram */
		ssize_t n = sendto(fd, &sample, sizeof(sample), 0,
				   (struct sockaddr *)&addr, sizeof(addr));
		if (n < 0) {
			fprintf(stderr, "[comm] sendto failed (#%d): %s\n",
				seq, strerror(errno));
		} else if (n != sizeof(sample)) {
			fprintf(stderr, "[comm] sendto partial: %zd/%zu\n",
				n, sizeof(sample));
		}
	}

	if (fd >= 0)
		close(fd);
	printf("[comm] Communication thread exiting (%d samples sent)\n", seq);
	return NULL;
}

/* ------------------------------------------------------------------ */
/*  Main program                                                      */
/* ------------------------------------------------------------------ */

int main(int argc, char *argv[])
{
	struct iio_context *ctx = NULL;
	struct iio_device  *dev = NULL;
	struct iio_channel *ch[NUM_CHANNELS] = {NULL};
	struct iio_buffer  *buf = NULL;
	struct iio_channels_mask *mask = NULL;
	struct iio_stream  *stream = NULL;

	int max_samples = 0;	/* 0 = run forever until signal */
	int sample_count = 0;
	int ret = 0;
	bool comm_started = false;
	pthread_t comm_thread;

	if (argc > 1) {
		max_samples = atoi(argv[1]);
		if (max_samples < 0)
			max_samples = 0;	/* 0 = run forever */
	}

	signal(SIGINT, sigint_handler);
	signal(SIGTERM, sigint_handler);

	/* ---- Create IIO context (local) ---- */
	ctx = iio_create_context(NULL, "local:");
	if (iio_err(ctx)) {
		fprintf(stderr, "ERROR: Cannot create local IIO context: %s\n",
			strerror(-iio_err(ctx)));
		return 1;
	}

	/*
	 * Set timeout: BIA ODR is 5Hz (200ms period). Use 5s timeout
	 * to tolerate startup delay and occasional late data.
	 * A value of 0 means no timeout (wait indefinitely).
	 */
	iio_context_set_timeout(ctx, 5000);

	/* ---- Find AD5940 device ---- */
	dev = iio_context_find_device(ctx, "ad5940");
	if (!dev) {
		fprintf(stderr, "ERROR: Cannot find IIO device 'ad5940'. "
			"Is the kernel module loaded?\n");
		ret = 1;
		goto cleanup;
	}

	printf("AD5940 BIA Impedance Demo (libiio v%u.%u)\n",
	       iio_context_get_version_major(ctx),
	       iio_context_get_version_minor(ctx));
	printf("Device: %s, Channels: %u\n",
	       iio_device_get_name(dev) ? iio_device_get_name(dev) : "ad5940",
	       iio_device_get_channels_count(dev));

	/* ---- Get channels ---- */
	ch[0] = iio_device_find_channel(dev, "current0", false);
	ch[1] = iio_device_find_channel(dev, "current1", false);
	ch[2] = iio_device_find_channel(dev, "voltage0", false);
	ch[3] = iio_device_find_channel(dev, "voltage1", false);
	ch[4] = iio_device_find_channel(dev, "altvoltage0", false);
	ch[5] = iio_device_find_channel(dev, "resistance0", false);
	ch[6] = iio_device_find_channel(dev, "phase0", false);

	for (int i = 0; i < NUM_CHANNELS; i++) {
		if (!ch[i]) {
			fprintf(stderr, "ERROR: Cannot find channel index %d\n", i);
			ret = 1;
			goto cleanup;
		}
	}

	/* ---- Get buffer (pre-allocated by kernel driver) ---- */
	buf = iio_device_get_buffer(dev, 0);
	if (iio_err(buf)) {
		fprintf(stderr, "ERROR: Cannot get IIO buffer (index 0): %s\n",
			strerror(-iio_err(buf)));
		ret = 1;
		goto cleanup;
	}

	/* ---- Create channels mask and enable channels ---- */
	mask = iio_create_channels_mask(iio_device_get_channels_count(dev));
	if (!mask) {
		fprintf(stderr, "ERROR: Cannot create channels mask\n");
		ret = 1;
		goto cleanup;
	}

	for (int i = 0; i < NUM_CHANNELS; i++)
		iio_channel_enable(ch[i], mask);

	/* ---- Create stream ---- */
	/*
	 * iio_buffer_create_stream: creates nb_blocks blocks, each holding
	 * samples_count samples. For 1-sample-per-block low-latency reads,
	 * use nb_blocks=4, samples_count=1.
	 */
	stream = iio_buffer_create_stream(buf, 4, 1, mask);
	if (iio_err(stream)) {
		fprintf(stderr, "ERROR: Cannot create IIO stream: %s\n",
			strerror(-iio_err(stream)));
		ret = 1;
		goto cleanup;
	}

	// printf("\nCollecting %s samples (Ctrl-C to stop)...\n",
	//        max_samples ? "up to N" : "continuous");
	// printf("RTIA calibrated, Frequency from driver\n\n");
	// printf("%-6s %10s %12s %12s %12s %12s %12s %12s %10s %10s  %s\n",
	//        "#", "Freq(Hz)", "|Z|(Ohm)", "Phase(deg)", "R(Ohm)", "X(Ohm)",
	//        "CurrMag", "VoltMag", "Rtia(Ohm)", "RtiaPh(d)", "CurrDFT(R/I) VoltDFT(R/I)");
	// printf("------ ---------- ------------ ------------ ------------ ------------ "
	//        "------------ ------------ ---------- ----------  -----------------------\n");

	/* ---- Start communication thread ---- */
	ret = pthread_create(&comm_thread, NULL, comm_thread_fn, NULL);
	if (ret) {
		fprintf(stderr, "ERROR: Cannot create comm thread: %s\n",
			strerror(ret));
		ret = 1;
		goto cleanup;
	}
	comm_started = true;

	/* ---- Main data acquisition loop ---- */
	while (keep_running) {
		const struct iio_block *block;

		block = iio_stream_get_next_block(stream);
		if (iio_err(block)) {
			if (!keep_running)
				break;
			fprintf(stderr, "Stream read error: %s\n",
				strerror(-iio_err(block)));
			ret = 1;
			break;
		}
		if (!block) {
			/* Should not happen, but be safe */
			break;
		}

		/*
		 * Extract raw channel data using iio_channel_read().
		 * raw=true: read samples in hardware format,
		 *           no scale/offset conversion by libiio.
		 * Channels 0-4 are 32-bit, channel 5 (RTIA mag) is 64-bit,
		 * channel 6 (RTIA phase) is 32-bit.
		 */
		int32_t raw[NUM_CHANNELS];
		int64_t raw_rtia_mag;
		int32_t raw_rtia_phase;

		for (int i = 0; i < 5; i++) {
			size_t nr = iio_channel_read(ch[i], block,
						     &raw[i], sizeof(int32_t),
						     true);
			if (nr < sizeof(int32_t)) {
				fprintf(stderr, "Channel %d: short read (%zu)\n",
					i, nr);
				break;
			}
		}
		/* RTIA magnitude: s64 milliohms */
		iio_channel_read(ch[5], block, &raw_rtia_mag,
				 sizeof(int64_t), true);
		/* RTIA phase: s32 millidegrees */
		iio_channel_read(ch[6], block, &raw_rtia_phase,
				 sizeof(int32_t), true);

		/* Sign-extend 18-bit DFT values */
		int32_t curr_real = sign_extend_18bit((uint32_t)raw[0]);
		int32_t curr_imag = sign_extend_18bit((uint32_t)raw[1]);
		int32_t volt_real = sign_extend_18bit((uint32_t)raw[2]);
		int32_t volt_imag = sign_extend_18bit((uint32_t)raw[3]);

		/* Frequency channel: unsigned 32-bit Hz value from driver */
		uint32_t freq_hz = (uint32_t)raw[4];

		/* RTIA calibrated values from driver */
		float rtia_mag_ohm = (float)raw_rtia_mag / 1000.0f;
		float rtia_phase_deg = (float)raw_rtia_phase / 1000.0f;

		/* Compute impedance with calibrated RTIA */
		bia_sample_t sample;
		compute_impedance(curr_real, curr_imag,
				  volt_real, volt_imag,
				  rtia_mag_ohm, rtia_phase_deg,
				  freq_hz, &sample);

		/* Diagnostic magnitudes */
		float curr_mag = sqrtf((float)curr_real * curr_real +
				       (float)curr_imag * curr_imag);
		float volt_mag = sqrtf((float)volt_real * volt_real +
				       (float)volt_imag * volt_imag);

		sample_count++;
		// printf("%-6d %10u %12.2f %12.2f %12.2f %12.2f %12.1f %12.1f %10.2f %10.2f  "
		//        "(%d/%d) (%d/%d)\n",
		//        sample_count, freq_hz, sample.magnitude, sample.phase,
		//        sample.resistance, sample.reactance,
		//        curr_mag, volt_mag,
		//        rtia_mag_ohm, rtia_phase_deg,
		//        curr_real, curr_imag, volt_real, volt_imag);

		/* Push to ring buffer for communication thread */
		ring_push(&g_ring, &sample);

		if (max_samples > 0 && sample_count >= max_samples)
			break;
	}

	printf("\n--- Summary ---\n");
	printf("Samples collected: %d\n", sample_count);
	if (sample_count > 0)
		printf("RTIA calibrated (values from driver)\n");

	/* Wake comm thread so it can exit */
	pthread_mutex_lock(&g_ring.mutex);
	pthread_cond_signal(&g_ring.cond);
	pthread_mutex_unlock(&g_ring.mutex);

	if (comm_started)
		pthread_join(comm_thread, NULL);

cleanup:
	if (stream)
		iio_stream_destroy(stream);
	if (mask)
		iio_channels_mask_destroy(mask);
	if (ctx)
		iio_context_destroy(ctx);

	return ret;
}
