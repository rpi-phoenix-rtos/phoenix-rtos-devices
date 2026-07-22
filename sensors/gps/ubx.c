/*
 * Phoenix-RTOS
 *
 * Driver for GPS module ubx
 *
 * Copyright 2022, 2026 Phoenix Systems
 * Author: Damian Loewnau, Mateusz Niewiadomski, Jakub Smolaga
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <sys/threads.h>
#include <errno.h>
#include <math/consts.h>

#include <libsensors/sensor.h>
#include <libsensors/gps/receiver.h>


#define UBX_STR "ubx:"

#define UBX_SYNC1   0xb5
#define UBX_SYNC2   0x62
#define UBX_BUFSZ   512
#define UBX_STKSZ   2048
#define UBX_MAXLEN  256
#define UBX_HDRLEN  6
#define UBX_CHKLEN  2
#define UBX_MINLEN  (UBX_HDRLEN + UBX_CHKLEN)
#define UBX_SYNCLEN 2

#define UBX_KEY_UART1_OUT_PROT_UBX   0x10740001
#define UBX_KEY_UART1_OUT_PROT_NMEA  0x10740002
#define UBX_KEY_UART1_IN_PROT_UBX    0x10730001
#define UBX_KEY_UART1_IN_PROT_NMEA   0x10730002
#define UBX_KEY_UART1_BAUDRATE       0x40520001
#define UBX_KEY_RATE_MEAS            0x30210001
#define UBX_KEY_RATE_NAV             0x30210002
#define UBX_KEY_RATE_TIMEREF         0x20210003
#define UBX_KEY_MSGOUT_NAV_PVT_UART1 0x20910007

#define UBX_NEVER_US (0x0fffffffffffffffLL)
#define UBX_POLL_US  (50LL * 1000LL)
#define UBX_TOUT_US  (500LL * 1000LL)

#define ubx_STR "ubx:"

/* Specified by the module's documentation */
#define UPDATE_RATE_MS 100

/* ubx has: pos_CEP = 3m, vel_CEP = 0.1m/s. We use RMS values which are 0.849 * CEP (https://en.wikipedia.org/wiki/Circular_error_probable) */
#define UBX_POS_ACCURACY 2.547f
#define UBX_VEL_ACCURACY 0.0849f

#define REC_BUF_SZ 512

#ifndef UBX_DEFAULT_BAUDRATE
#define UBX_DEFAULT_BAUDRATE 115200
#endif


typedef uint8_t ubx_class_t;
enum {
	ubx_class_nav = 0x01,
	ubx_class_ack = 0x05,
	ubx_class_cfg = 0x06,
	ubx_class_mon = 0x0A,
};

typedef uint8_t ubx_cfgLayer_t;
enum {
	ubx_cfgLayer_ram = (1 << 0),
	ubx_cfgLayer_bbr = (1 << 1),
	ubx_cfgLayer_flash = (1 << 2),
};

typedef uint8_t ubx_hwVer_t;
enum {
	ubx_hwVer_undef = 0,
	ubx_hwVer_m5,
	ubx_hwVer_m6,
	ubx_hwVer_m7,
	ubx_hwVer_m8,
	ubx_hwVer_m9,
	ubx_hwVer_m10
};

typedef uint8_t ubx_fix_t;
enum {
	ubx_fix_none = 0,      /* no fix */
	ubx_fix_dr = 0x01,     /* dead reckoning only */
	ubx_fix_2d = 0x02,     /* 2d fix */
	ubx_fix_3d = 0x03,     /* 3d fix */
	ubx_fix_gnssDr = 0x04, /* gnss + dead reckoning combined */
	ubx_fix_time = 0x05,   /* time only fix */
};

typedef struct {
	uint8_t buf[UBX_BUFSZ];
	size_t wrIdx;
	size_t rdIdx;
} ubx_parser_t;

typedef struct {
	uint8_t buf[UBX_BUFSZ];
	size_t pos;
} ubx_builder_t;

typedef struct {
	uint8_t *pld;
	uint16_t len;
	ubx_class_t class;
	uint8_t id;
} ubx_msg_t;

typedef struct {
	sensor_event_t evtGps;
	int tid;
	int fd;
	ubx_parser_t p;
	ubx_builder_t b;
	ubx_msg_t msg;
	ubx_hwVer_t ver;
	volatile bool run;
	__attribute__((aligned(8))) uint8_t stk[UBX_STKSZ];
} ubx_ctx_t;

static bool ubx_isDeadlineDue(time_t deadlineUs)
{
	time_t nowUs = 0;
	gettime(&nowUs, NULL);
	return nowUs >= deadlineUs;
}


static time_t ubx_getDeadline(time_t toutUs)
{
	time_t nowUs = 0;
	gettime(&nowUs, NULL);
	return nowUs + toutUs;
}


/* accumulate checksum using the Fletcher algorithm */
static void ubx_chk(const uint8_t *buf, size_t bufsz, uint8_t *a, uint8_t *b)
{
	for (size_t i = 0; i < bufsz; i++) {
		*a += buf[i];
		*b += *a;
	}
}


/* deg * 1e-5 to mrad */
// static uint64_t ubx_degE5ToMRad(uint64_t deg)
// {
// 	static const uint64_t scaler = (uint64_t)((1e9L / 360.0) * 2.0 * M_PI);
// 	static const uint64_t divider = (uint64_t)(1e6L);
// 	return (deg * scaler) / divider;
// }


/* convert ubx fix to nmea fix used by libsensors */
uint8_t ubx_getNmeaGGAFix(ubx_fix_t fix)
{
	switch (fix) {
		case ubx_fix_2d:
		case ubx_fix_3d:
			return gga_fix_gnss;

		case ubx_fix_dr:
		case ubx_fix_gnssDr:
			return gga_fix_estimated;

		default:
			return gga_fix_invalid;
	}
}


/********************************** parsing ***********************************/


/*
 * tries to find a message
 * if message was not found - returns false
 * if message was found - returns true and writes data to msg
 */
static bool ubx_parse(ubx_parser_t *p, ubx_msg_t *msg)
{
	for (;;) {
		/* don't try parsing if there is no chance for a full frame */
		unsigned avail = p->wrIdx - p->rdIdx;
		if (avail < UBX_MINLEN) {
			return false;
		}

		/* extract header */
		uint8_t sync1 = p->buf[p->rdIdx + 0];
		uint8_t sync2 = p->buf[p->rdIdx + 1];
		uint8_t class = p->buf[p->rdIdx + 2];
		uint8_t msgId = p->buf[p->rdIdx + 3];
		uint8_t lenLo = p->buf[p->rdIdx + 4];
		uint8_t lenHi = p->buf[p->rdIdx + 5];
		uint16_t len = (uint16_t)lenLo | ((uint16_t)lenHi << 8);

		/* check sync bytes */
		if (sync1 != UBX_SYNC1 || sync2 != UBX_SYNC2) {
			p->rdIdx++;
			continue;
		}

		/* reject messages which are too long to parse */
		if (len > UBX_MAXLEN - UBX_HDRLEN - UBX_CHKLEN) {
			p->rdIdx++;
			continue;
		}

		/* return false if we don't have enough data yet */
		if (len > avail - UBX_HDRLEN - UBX_CHKLEN) {
			return false;
		}

		/* calculate checksum */
		uint8_t *chkBeg = p->buf + p->rdIdx + UBX_SYNCLEN;
		uint8_t *chkEnd = p->buf + p->rdIdx + UBX_HDRLEN + len;
		uint8_t chkA = 0;
		uint8_t chkB = 0;
		ubx_chk(chkBeg, chkEnd - chkBeg, &chkA, &chkB);

		/* verify checksum */
		if (chkA != chkEnd[0] || chkB != chkEnd[1]) {
			p->rdIdx++;
			continue;
		}

		/* copy data to the output buffer */
		msg->pld = p->buf + p->rdIdx + UBX_HDRLEN;
		msg->len = len;
		msg->class = class;
		msg->id = msgId;
		p->rdIdx += UBX_HDRLEN + len + UBX_CHKLEN;

		/* notify caller that we found a message */
		return true;
	}
}


static ssize_t ubx_read(ubx_ctx_t *ctx, time_t deadlineUs)
{
	uint8_t *buf = ctx->p.buf + ctx->p.wrIdx;
	size_t bufsz = sizeof(ctx->p.buf) - ctx->p.wrIdx;

	for (;;) {
		ssize_t count = read(ctx->fd, buf, bufsz);

		if (count > 0) {
			ctx->p.wrIdx += count;
			return count;
		}

		if (ubx_isDeadlineDue(deadlineUs)) {
			return -1;
		}

		usleep(UBX_POLL_US);
	}
}


static void ubx_parserCompact(ubx_parser_t *p)
{
	/* move read/write heads to the front if there is no data left */
	if (p->wrIdx <= p->rdIdx) {
		p->wrIdx = 0;
		p->rdIdx = 0;
	}

	/* compact the buffer if it's too far */
	if (p->wrIdx + UBX_MAXLEN > UBX_BUFSZ) {
		unsigned leftover = p->wrIdx - p->rdIdx;
		memmove(p->buf, p->buf + p->rdIdx, leftover);
		p->wrIdx = leftover;
		p->rdIdx = 0;
	}
}


static int ubx_msgGet(ubx_ctx_t *ctx, time_t deadlineUs)
{
	for (;;) {
		/* try to parse what is already in the buffer */
		if (ubx_parse(&ctx->p, &ctx->msg)) {
			return 0;
		}

		/* make sure we have enough space for the read */
		ubx_parserCompact(&ctx->p);

		/* try to read more data */
		ssize_t count = ubx_read(ctx, deadlineUs);
		if (count < 0) {
			return -1;
		}
	}
}


static int ubx_msgGetByClass(ubx_ctx_t *ctx, ubx_class_t cls, time_t deadlineUs)
{
	for (;;) {
		if (ubx_msgGet(ctx, deadlineUs) < 0) {
			return -1;
		}
		if (ctx->msg.class == cls) {
			return 0;
		}
	}
}


static int ubx_msgGetExact(ubx_ctx_t *ctx, ubx_class_t cls, uint8_t id, time_t deadlineUs)
{
	for (;;) {
		if (ubx_msgGetByClass(ctx, cls, deadlineUs) < 0) {
			return -1;
		}
		if (ctx->msg.id == id) {
			return 0;
		}
	}
}


/* wait for ack or immediately fail if nak was received */
static int ubx_msgGetAck(ubx_ctx_t *ctx, time_t deadlineUs)
{
	while (ctx->run) {
		int ret = ubx_msgGetByClass(ctx, ubx_class_ack, deadlineUs);
		if (ret < 0) {
			return ret;
		}
		/* UBX-ACK-ACK=0x01, UBX-ACK-NAK=0x00 */
		if (ctx->msg.id == 0x01) {
			return 0;
		}
		else {
			return -1;
		}
	}

	return -1;
}


/****************************** message builder *******************************/


static void ubx_push(ubx_ctx_t *ctx, const void *data, size_t sz)
{
	/* always reserve space for the header */
	ctx->b.pos = (ctx->b.pos < UBX_HDRLEN) ? UBX_HDRLEN : ctx->b.pos;

	/* check if data is ok and will fit */
	if (sz == 0 || data == NULL || ctx->b.pos + sz >= UBX_BUFSZ) {
		return;
	}

	/* copy the data */
	memcpy(ctx->b.buf + ctx->b.pos, data, sz);
	ctx->b.pos += sz;
}


static inline void ubx_pushU8(ubx_ctx_t *ctx, uint8_t data)
{
	ubx_push(ctx, &data, sizeof(data));
}


static inline void ubx_pushU16(ubx_ctx_t *ctx, uint16_t data)
{
	ubx_pushU8(ctx, (uint8_t)((data >> 000) & 0xff));
	ubx_pushU8(ctx, (uint8_t)((data >> 010) & 0xff));
}


static inline void ubx_pushU32(ubx_ctx_t *ctx, uint32_t data)
{
	ubx_pushU16(ctx, (uint16_t)((data >> 000) & 0xffff));
	ubx_pushU16(ctx, (uint16_t)((data >> 020) & 0xffff));
}


static inline void ubx_pushU64(ubx_ctx_t *ctx, uint64_t data)
{
	ubx_pushU32(ctx, (uint32_t)((data >> 000) & 0xffffffff));
	ubx_pushU32(ctx, (uint32_t)((data >> 040) & 0xffffffff));
}


static inline void ubx_pushValSetHdr(ubx_ctx_t *ctx, ubx_cfgLayer_t layer)
{
	ubx_pushU8(ctx, 0x00);    /* message version */
	ubx_pushU8(ctx, layer);   /* configuration layer(s) */
	ubx_pushU16(ctx, 0x0000); /* reserved */
}


static inline void ubx_pushValSetItem(ubx_ctx_t *ctx, uint32_t key, uint64_t val)
{
	ubx_pushU32(ctx, key);
	uint8_t sizeBits = (uint8_t)((key >> 28) & 0x7);
	if (sizeBits == 0x05) {
		ubx_pushU64(ctx, val);
	}
	else if (sizeBits == 0x04) {
		ubx_pushU32(ctx, val);
	}
	else if (sizeBits == 0x03) {
		ubx_pushU16(ctx, val);
	}
	else {
		ubx_pushU8(ctx, val);
	}
}


static int ubx_send(ubx_ctx_t *ctx, ubx_class_t cls, uint8_t id)
{
	/* always reserve space for the header */
	ctx->b.pos = (ctx->b.pos < UBX_HDRLEN) ? UBX_HDRLEN : ctx->b.pos;

	/* check if the checksum will fit */
	if (ctx->b.pos + UBX_CHKLEN >= UBX_BUFSZ) {
		return -1;
	}

	/* calculate payload length */
	size_t pldLen = ctx->b.pos - UBX_HDRLEN;

	/* construct the header */
	ctx->b.buf[0] = UBX_SYNC1;
	ctx->b.buf[1] = UBX_SYNC2;
	ctx->b.buf[2] = cls;
	ctx->b.buf[3] = id;
	ctx->b.buf[4] = (uint8_t)((pldLen >> 000) & 0xff);
	ctx->b.buf[5] = (uint8_t)((pldLen >> 010) & 0xff);

	/* calculate checksum */
	uint8_t *chkA = ctx->b.buf + ctx->b.pos + 0;
	uint8_t *chkB = ctx->b.buf + ctx->b.pos + 1;
	*chkA = 0x00;
	*chkB = 0x00;
	ubx_chk(ctx->b.buf + 2, ctx->b.pos - 2, chkA, chkB);

	/* calculate total amount of data to be sent */
	size_t total = ctx->b.pos + UBX_CHKLEN;

	/* reset the builder */
	ctx->b.pos = UBX_HDRLEN;

	/* write the bytes */
	size_t sent = 0;
	while (sent < total) {
		ssize_t n = write(ctx->fd, ctx->b.buf + sent, total - sent);
		if (n < 0) {
			return -1;
		}
		sent += n;
	}

	/* return number of bytes sent */
	return sent;
}


/* auto-configuration for M9+ devices */
static int ubx_configureM9(ubx_ctx_t *ctx, uint32_t baud)
{
	int ret = 0;

	/* construct the configuration message */
	ubx_pushValSetHdr(ctx, ubx_cfgLayer_ram);
	ubx_pushValSetItem(ctx, UBX_KEY_UART1_OUT_PROT_UBX, 1);   /* enable ubx output */
	ubx_pushValSetItem(ctx, UBX_KEY_UART1_OUT_PROT_NMEA, 0);  /* disable nmea output */
	ubx_pushValSetItem(ctx, UBX_KEY_RATE_MEAS, 100);          /* measurement rate (100ms=10Hz) */
	ubx_pushValSetItem(ctx, UBX_KEY_RATE_NAV, 1);             /* 1 solution per measurement */
	ubx_pushValSetItem(ctx, UBX_KEY_RATE_TIMEREF, 1);         /* use gnss as time reference */
	ubx_pushValSetItem(ctx, UBX_KEY_MSGOUT_NAV_PVT_UART1, 1); /* enable nav-pvt messages */
	ubx_pushValSetItem(ctx, UBX_KEY_UART1_BAUDRATE, baud);    /* set baudrate */

	ret = ubx_send(ctx, ubx_class_cfg, 0x8a);
	if (ret < 0) {
		return ret;
	}

	ret = ubx_msgGetAck(ctx, ubx_getDeadline(UBX_TOUT_US));
	if (ret < 0) {
		return ret;
	}

	ret = gps_serialSetup(ctx->fd, baud, NULL);
	if (ret < 0) {
		return ret;
	}

	return 0;
}


static int ubx_configure(ubx_ctx_t *ctx, uint32_t baud)
{
	if (ctx->ver >= ubx_hwVer_m9) {
		return ubx_configureM9(ctx, baud);
	}
	else {
		assert(0 && "TODO");
		return -1;
	}
}


static int ubx_setup(ubx_ctx_t *ctx)
{
	/* list of baudrate options from most common to least common */
	static const unsigned baudRates[] = {
		B38400,  /* default configuration on modern devices */
		B9600,   /* default configuration on older devices */
		B115200, /* default on some high-end devices */
		B57600,  /* Custom */
		B19200,  /* Custom */
		B230400, /* Custom */
		B4800,   /* ancient nmea standard, too slow for most devices */
	};
	static const char *const hwVerStr[] = {
		[ubx_hwVer_undef] = "",
		[ubx_hwVer_m5] = "00040005",
		[ubx_hwVer_m6] = "00040007",
		[ubx_hwVer_m7] = "00070000",
		[ubx_hwVer_m8] = "00080000",
		[ubx_hwVer_m9] = "00190000",
		[ubx_hwVer_m10] = "000A0000",
	};
	static const size_t baudRatesCount = sizeof(baudRates) / sizeof(*baudRates);
	static const size_t hwVerStrCount = sizeof(hwVerStr) / sizeof(*hwVerStr);

	for (size_t i = 0; i < baudRatesCount; i++) {
		unsigned baud = baudRates[i];

		/* configure device */
		if (gps_serialSetup(ctx->fd, baud, NULL) < 0) {
			continue;
		}

		/* restart the parser */
		ctx->p.wrIdx = 0;
		ctx->p.rdIdx = 0;

		/* send version request */
		if (ubx_send(ctx, ubx_class_mon, 0x04) < 0) {
			continue;
		}

		/* wait for response */
		if (ubx_msgGetExact(ctx, ubx_class_mon, 0x04, ubx_getDeadline(UBX_TOUT_US)) < 0) {
			continue;
		}

		ctx->ver = ubx_hwVer_undef;
		for (size_t v = ubx_hwVer_undef; v < hwVerStrCount; v++) {
			if (strcmp((char *)(ctx->msg.pld + 30), hwVerStr[v]) == 0) {
				ctx->ver = v;
				break;
			}
		}

		if (ubx_configure(ctx, UBX_DEFAULT_BAUDRATE) < 0) {
			return -1;
		}

		return 0;
	}

	return -1;
}


static uint8_t ubx_readU8(uint8_t **p)
{
	uint8_t result = **p;
	*p += 1;
	return result;
}

static uint16_t ubx_readU16(uint8_t **p)
{
	uint8_t lo = ubx_readU8(p);
	uint8_t hi = ubx_readU8(p);
	return (((uint16_t)lo) << 000) | (((uint16_t)hi) << 010);
}


static uint32_t ubx_readU32(uint8_t **p)
{
	uint16_t lo = ubx_readU16(p);
	uint16_t hi = ubx_readU16(p);
	return (((uint32_t)lo) << 000) | (((uint32_t)hi) << 020);
}


static int16_t ubx_readI16(uint8_t **p)
{
	return (int16_t)ubx_readU16(p);
}


static int32_t ubx_readI32(uint8_t **p)
{
	return (int32_t)ubx_readU32(p);
}


static void ubx_threadPublish(void *data)
{
	/* calculate time needed to fill 50 % of buffer to set it as read interval */
	/* static const unsigned int sleepPeriod = ((REC_BUF_SZ / 2) * 1e6) / (UBX_DEFAULT_BAUDRATE / 10); */

	sensor_info_t *info = (sensor_info_t *)data;
	struct __errno_t errnoNew;
	ubx_ctx_t *ctx = info->ctx;
	int ret = 0;

	while (ubx_setup(ctx) != 0) {
		usleep(1e6);
	}

	/* reset UBX parser context */
	memset(&ctx->p, 0, sizeof(ubx_parser_t));

	/* Redirecting errno to keep backward compatibility (in case of errno not working correctly) */
	_errno_new(&errnoNew);

	while (ctx->run) {
		ret = ubx_msgGetExact(ctx, ubx_class_nav, 0x07, UBX_NEVER_US);
		if (ret < 0) {
			continue;
		}

		/* parse the payload */
		uint8_t *p = ctx->msg.pld;
		(void)ubx_readU32(&p);              /* iTOW */
		uint16_t year = ubx_readU16(&p);    /* year */
		uint8_t month = ubx_readU8(&p);     /* month [1; 12] */
		uint8_t day = ubx_readU8(&p);       /* day of month [1; 31] */
		uint8_t hour = ubx_readU8(&p);      /* hour of day [0; 32] */
		uint8_t min = ubx_readU8(&p);       /* minute of hour [0; 59] */
		uint8_t sec = ubx_readU8(&p);       /* second of minute [0; 60] */
		(void)ubx_readU8(&p);               /* validity flags */
		(void)ubx_readU32(&p);              /* time accuracy estimate */
		int32_t nano = ubx_readI32(&p);     /* fraction of second [-1e9; +1e9] */
		ubx_fix_t fix = ubx_readU8(&p);     /* gnss fix type */
		(void)ubx_readU8(&p);               /* fix status flags */
		(void)ubx_readU8(&p);               /* additional flags */
		uint8_t numSV = ubx_readU8(&p);     /* number of satellites used */
		int32_t lon = ubx_readI32(&p);      /* longitude [deg * 1e-7] */
		int32_t lat = ubx_readI32(&p);      /* latitude [deg * 1e-7] */
		int32_t height = ubx_readI32(&p);   /* height above ellipsoid [mm] */
		int32_t hMSL = ubx_readI32(&p);     /* height above mean sea level [mm] */
		uint32_t hAcc = ubx_readU32(&p);    /* horizontal accuracy estimate [mm] */
		uint32_t vAcc = ubx_readU32(&p);    /* vertical accuracy estimate [mm] */
		int32_t velN = ubx_readI32(&p);     /* ned north velocity [mm/s] */
		int32_t velE = ubx_readI32(&p);     /* ned east velocity [mm/s] */
		int32_t velD = ubx_readI32(&p);     /* ned down velocity [mm/s] */
		int32_t gSpeed = ubx_readI32(&p);   /* ground speed 2d [mm/s] */
		int32_t headMot = ubx_readI32(&p);  /* heading of motion 2d [deg * 1e-5] */
		uint32_t sAcc = ubx_readU32(&p);    /* speed accuracy estimate [mm/s] */
		uint32_t headAcc = ubx_readU32(&p); /* heading accuracy estimate [deg * 1e-5] */
		(void)ubx_readU16(&p);              /* position dop [1e-2] */
		(void)ubx_readU16(&p);              /* additional flags */
		(void)ubx_readU32(&p);              /* reserved */
		(void)ubx_readU16(&p);              /* heading of vehicle [deg * 1e-5] */
		int16_t magDec = 0;                 /* magnetic declination [deg * 1e-2] */
		if (ctx->msg.len >= 92) {
			magDec = ubx_readI16(&p);
		}

		printf(">> GOT FIX: %d\n", fix);
		printf(">> %d-%d-%d %d:%d\n", year, month, day, hour, min);
		printf(">> lat: %g, lon: %g\n", lat * 1e-7f, lon * 1e-7f);

		/* convert time */
		struct tm tmUTC = {
			.tm_year = year - 1900,
			.tm_mon = month - 1,
			.tm_mday = day,
			.tm_hour = hour,
			.tm_min = min,
			.tm_sec = sec,
			.tm_isdst = 0,
		};
		time_t epochSec = timegm(&tmUTC);
		uint64_t epochUs = ((uint64_t)epochSec * 1000000ULL) + (nano / 1000ULL);

		/* construct data */
		ctx->evtGps.gps = (gps_data_t) {
			.utc = epochUs,
			.fix = ubx_getNmeaGGAFix(fix),
			.satsNb = numSV,
			.lon = lon * 100LL,
			.lat = lat * 100LL,
			.altEllipsoid = height,
			.alt = hMSL,
			.eph = hAcc,
			.epv = vAcc,
			.velNorth = velN,
			.velEast = velE,
			.velDown = velD,
			.groundSpeed = (uint32_t)gSpeed,
			.evel = sAcc,
			.heading = (uint16_t)((headMot * 17453LL) / 100000000LL),
			.headingAccur = (uint16_t)((headAcc * 17453LL) / 100000000LL),
			.hdop = 0,
			.vdop = 0,
			.headingOffs = (int16_t)((magDec * 17453LL) / 100000LL),
		};

		sensors_publish(SENSOR_TYPE_GPS, &ctx->evtGps);
	}
}


static int ubx_start(sensor_info_t *info)
{
	int err;
	ubx_ctx_t *ctx = info->ctx;

	ctx->evtGps.type = SENSOR_TYPE_GPS;
	ctx->evtGps.gps.devId = info->id;
	ctx->run = 1;

	err = beginthreadex(ubx_threadPublish, THREAD_PRIORITY_SENSOR, ctx->stk, UBX_STKSZ, info, &ctx->tid);
	if (err < 0) {
		free(ctx);
	}
	else {
		printf("%s launched sensor\n", UBX_STR);
	}

	return err;
}


static int ubx_parseArgs(const char *args, const char **path)
{
	int err = 0;

	if (!(args == NULL || strchr(args, ':'))) {
		*path = args;
	}
	else {
		fprintf(stderr,
				"%s Wrong arguments\n"
				"Please specify the path to source device instance, for example: /dev/uart0\n",
				UBX_STR);
		err = -EINVAL;
	}

	return err;
}


static int ubx_alloc(sensor_info_t *info, const char *args)
{
	int cnt = 0, err = -1;
	const char *path;
	ubx_ctx_t *ctx;

	ctx = malloc(sizeof(ubx_ctx_t));
	if (ctx == NULL) {
		return -ENOMEM;
	}

	err = ubx_parseArgs(args, &path);
	if (err != EOK) {
		free(ctx);
		return err;
	}

	/* Opening serial device */
	do {
		ctx->fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (ctx->fd < 0) {
			usleep(10 * 1000);
			cnt++;

			if (cnt > 10000) {
				fprintf(stderr, "%s Can't open %s: %s\n", UBX_STR, path, strerror(errno));
				err = -errno;
				free(ctx);
				return err;
			}
		}
	} while (ctx->fd < 0);

	if (isatty(ctx->fd) != 1) {
		fprintf(stderr, "%s %s not a tty\n", UBX_STR, path);
		close(ctx->fd);
		free(ctx);
		return -1;
	}

	info->ctx = ctx;

	return 0;
}


static int ubx_dealloc(sensor_info_t *info)
{
	ubx_ctx_t *ctx;

	if (!info || !info->ctx) {
		return -EINVAL;
	}

	ctx = (ubx_ctx_t *)info->ctx;
	ctx->run = 0;

	(void)close(ctx->fd);

	if (ctx->tid) {
		(void)threadJoin(ctx->tid, (time_t)-1);
	}

	free(ctx);
	info->ctx = NULL;

	return 0;
}


void __attribute__((constructor)) ubx_register(void)
{
	static sensor_drv_t sensor = {
		.name = "ubx",
		.alloc = ubx_alloc,
		.start = ubx_start,
		.dealloc = ubx_dealloc,
	};

	sensors_register(&sensor);
}
