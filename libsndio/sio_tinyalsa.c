/*	$OpenBSD$	*/
/*
 * Copyright (c) 2008 Alexandre Ratchov <alex@caoua.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifdef USE_TINYALSA
#include <sys/types.h>

#include <errno.h>
#include <limits.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <tinyalsa/pcm.h>

#include "debug.h"
#include "sio_priv.h"
#include "bsd-compat.h"

#ifdef DEBUG
#define DTINYALSA(str, pcm) fprintf(stderr, "%s: %s\n", str, pcm_get_error(pcm))
#else
#define DTINYALSA(str, pcm) do {} while (0)
#endif

struct sio_tinyalsa_hdl {
	struct sio_hdl sio;
	struct sio_par par;
	struct pcm *opcm;
	struct pcm *ipcm;
	int ifd, ofd;
	int card;
	int device;
 	unsigned int ibpf, obpf;	/* bytes per frame */
	int iused, oused;		/* frames used in hardware fifos */
	int idelta, odelta;		/* position reported to client */
	int running;
	int events;
	int ipartial, opartial;
	char *itmpbuf, *otmpbuf;
};

static void sio_tinyalsa_close(struct sio_hdl *);
static int sio_tinyalsa_start(struct sio_hdl *);
static int sio_tinyalsa_stop(struct sio_hdl *);
static int sio_tinyalsa_setpar(struct sio_hdl *, struct sio_par *);
static int sio_tinyalsa_getpar(struct sio_hdl *, struct sio_par *);
static int sio_tinyalsa_getcap(struct sio_hdl *, struct sio_cap *);
static size_t sio_tinyalsa_read(struct sio_hdl *, void *, size_t);
static size_t sio_tinyalsa_write(struct sio_hdl *, const void *, size_t);
static int sio_tinyalsa_nfds(struct sio_hdl *);
static int sio_tinyalsa_pollfd(struct sio_hdl *, struct pollfd *, int);
static int sio_tinyalsa_revents(struct sio_hdl *, struct pollfd *);

static struct sio_ops sio_tinyalsa_ops = {
	sio_tinyalsa_close,
	sio_tinyalsa_setpar,
	sio_tinyalsa_getpar,
	sio_tinyalsa_getcap,
	sio_tinyalsa_write,
	sio_tinyalsa_read,
	sio_tinyalsa_start,
	sio_tinyalsa_stop,
	sio_tinyalsa_nfds,
	sio_tinyalsa_pollfd,
	sio_tinyalsa_revents,
	NULL, /* setvol */
	NULL, /* getvol */
};

#define SNDRV_PCM_FORMAT_S8 0
#define SNDRV_PCM_FORMAT_S16_LE 2
#define SNDRV_PCM_FORMAT_S16_BE 3
#define SNDRV_PCM_FORMAT_S24_LE 6
#define SNDRV_PCM_FORMAT_S24_BE 7
#define SNDRV_PCM_FORMAT_S32_LE 10
#define SNDRV_PCM_FORMAT_S32_BE 11
#define SNDRV_PCM_FORMAT_ANY			\
	((1 << SNDRV_PCM_FORMAT_S8)     |	\
	 (1 << SNDRV_PCM_FORMAT_S16_LE) |	\
	 (1 << SNDRV_PCM_FORMAT_S16_BE) |	\
	 (1 << SNDRV_PCM_FORMAT_S24_LE) |	\
	 (1 << SNDRV_PCM_FORMAT_S24_BE) |	\
	 (1 << SNDRV_PCM_FORMAT_S32_LE) |	\
	 (1 << SNDRV_PCM_FORMAT_S32_BE))


#define CAP_NFMTS	(sizeof(cap_fmts)  / sizeof(cap_fmts[0]))
#define CAP_NCHANS	(sizeof(cap_chans) / sizeof(cap_chans[0]))
#define CAP_NRATES	(sizeof(cap_rates) / sizeof(cap_rates[0]))

static unsigned int cap_chans[] = {
	1, 2, 4, 6, 8, 10, 12, 16
};
static unsigned int cap_rates[] = {
	 8000, 11025, 12000, 16000, 22050, 24000,
	32000, 44100, 48000, 64000, 88200, 96000
};
static enum pcm_format cap_fmts[] = {
	PCM_FORMAT_S32_LE,	PCM_FORMAT_S32_BE,
	PCM_FORMAT_S24_LE,	PCM_FORMAT_S24_BE,
	PCM_FORMAT_S16_LE,	PCM_FORMAT_S16_BE,
	PCM_FORMAT_S8
};

/*
 * convert ALSA format to sio_par encoding
 */
static int
sio_tinyalsa_fmttopar(struct sio_tinyalsa_hdl *hdl, enum pcm_format fmt,
    unsigned int *bits, unsigned int *sig, unsigned int *le)
{
	switch (fmt) {
	case PCM_FORMAT_S8:
		*bits = 8;
		*sig = 1;
		break;
	case PCM_FORMAT_S16_LE:
		*bits = 16;
		*sig = 1;
		*le = 1;
		break;
	case PCM_FORMAT_S16_BE:
		*bits = 16;
		*sig = 1;
		*le = 0;
		break;
	case PCM_FORMAT_S24_LE:
		*bits = 24;
		*sig = 1;
		*le = 1;
		break;
	case PCM_FORMAT_S24_BE:
		*bits = 24;
		*sig = 1;
		*le = 0;
		break;
	case PCM_FORMAT_S32_LE:
		*bits = 32;
		*sig = 1;
		*le = 1;
		break;
	case PCM_FORMAT_S32_BE:
		*bits = 32;
		*sig = 1;
		*le = 0;
		break;
	default:
		DPRINTF("sio_tinyalsa_fmttopar: 0x%x: unsupported format\n", fmt);
		hdl->sio.eof = 1;
		return 0;
	}
	return 1;
}


/*
 * convert sio_par encoding to ALSA format
 */
static void
sio_alsa_enctofmt(struct sio_tinyalsa_hdl *hdl, enum pcm_format *rfmt,
    unsigned int bits, unsigned int sig, unsigned int le)
{
	if (bits == 8) {
		*rfmt = PCM_FORMAT_S8;
	} else if (bits == 16) {
		if (le == ~0U) {
			*rfmt = SIO_LE_NATIVE ?
				PCM_FORMAT_S16_LE :
				PCM_FORMAT_S16_BE;
		} else if (le)
			*rfmt = PCM_FORMAT_S16_LE;
		else
			*rfmt = PCM_FORMAT_S16_BE;
	} else if (bits == 24) {
		if (le == ~0U) {
			*rfmt = SIO_LE_NATIVE ?
				PCM_FORMAT_S24_LE :
				PCM_FORMAT_S24_BE;
		 } else if (le)
			*rfmt = PCM_FORMAT_S24_LE;
		else
			*rfmt = PCM_FORMAT_S24_BE;
	} else if (bits == 32) {
		if (le == ~0U) {
			*rfmt = SIO_LE_NATIVE ?
				PCM_FORMAT_S32_LE :
				PCM_FORMAT_S32_BE;
		 } else if (le)
			*rfmt = PCM_FORMAT_S32_LE;
		else
			*rfmt = PCM_FORMAT_S32_BE;
	} else {
		*rfmt = SIO_LE_NATIVE ?
		    PCM_FORMAT_S16_LE : PCM_FORMAT_S16_BE;
	}
}

/*
 * guess device capabilities
 */
static int
sio_tinyalsa_getcap(struct sio_hdl *sh, struct sio_cap *cap)
{
	struct sio_tinyalsa_hdl *hdl = (struct sio_tinyalsa_hdl *)sh;
	struct pcm_params *params;
	const struct pcm_mask *mask;
	int fmts = 0, rates = 0, chans = 0;  
	unsigned int min, max;
	unsigned int m;
	int i;

	DPRINTFN(1, "sio_tinyalsa_getcap:\n");

	params = pcm_params_get(hdl->card, hdl->device, PCM_OUT);
	if (params == NULL) {
		DPERROR("AUDIO_GETPAR");
		hdl->sio.eof = 1;
		return 0;
	}

	mask = pcm_params_get_mask(params, PCM_PARAM_FORMAT);
	m = (mask->bits[1] << 8) | mask->bits[0];
	if (!(m & SNDRV_PCM_FORMAT_ANY))
		return 0;

	if (m & (1 << SNDRV_PCM_FORMAT_S32_LE))
		fmts |= 1 << 0;
	if (m & (1 << SNDRV_PCM_FORMAT_S32_BE))
		fmts |= 1 << 1;
	if (m & (1 << SNDRV_PCM_FORMAT_S24_LE))
		fmts |= 1 << 2;
	if (m & (1 << SNDRV_PCM_FORMAT_S24_BE))
		fmts |= 1 << 3;
	if (m & (1 << SNDRV_PCM_FORMAT_S16_LE))
		fmts |= 1 << 4;
	if (m & (1 << SNDRV_PCM_FORMAT_S16_BE))
		fmts |= 1 << 5;
	if (m & (1 << SNDRV_PCM_FORMAT_S8))
		fmts |= 1 << 6;

	min = pcm_params_get_min(params, PCM_PARAM_RATE);
	max = pcm_params_get_max(params, PCM_PARAM_RATE);
	for (i = 0; i < CAP_NRATES; i++) {
		if (cap_rates[i] >= min && cap_rates[i] <= max)
			rates |= 1 << i;
	}

	min = pcm_params_get_min(params, PCM_PARAM_CHANNELS);
	max = pcm_params_get_max(params, PCM_PARAM_CHANNELS);
	for (i = 0; i < CAP_NCHANS; i++) {
		if (cap_chans[i] >= min && cap_chans[i] <= max)
			chans |= 1 << i;
	}

	return 1;
}

struct sio_hdl *
_sio_tinyalsa_open(const char *str, unsigned int mode, int nbio)
{
	const char *p;
	struct sio_tinyalsa_hdl *hdl;
	int fd;

	p = _sndio_parsetype(str, "rsnd");
	if (p == NULL) {
		DPRINTF("_sio_tinyalsa_open: %s: \"rsnd\" expected\n", str);
		return NULL;
	}
	switch (*p) {
	case '/':
		p++;
		break;
	default:
		DPRINTF("_sio_tinyalsa_open: %s: '/' expected\n", str);
		return NULL;
	}
	hdl = malloc(sizeof(struct sio_tinyalsa_hdl));
	if (hdl == NULL)
		return NULL;
	_sio_create(&hdl->sio, &sio_tinyalsa_ops, mode, nbio);

	hdl->card = 3;
	hdl->device = 0;
	hdl->opcm = pcm_open_by_name(p, PCM_OUT | PCM_NONBLOCK, NULL);
	if (!pcm_is_ready(hdl->opcm)) {
		DTINYALSA("pcm_is_ready", hdl->opcm);
		return NULL;
	}
	hdl->ofd = pcm_get_file_descriptor(hdl->opcm);
	DPRINTF("pcm_get_file_descriptor: %d\n", hdl->ofd);

	return (struct sio_hdl *)hdl;
#if 0
	fd = sio_tinyalsa_getfd(str, mode, nbio);
	if (fd < 0)
		return NULL;
	hdl = sio_tinyalsa_fdopen(fd, mode, nbio);
	if (hdl != NULL)
		return hdl;
	while (close(fd) < 0 && errno == EINTR)
		; /* retry */
#endif
	return NULL;
}

static void
sio_tinyalsa_close(struct sio_hdl *sh)
{
	struct sio_tinyalsa_hdl *hdl = (struct sio_tinyalsa_hdl *)sh;
	if (hdl->opcm)
		pcm_close(hdl->opcm);
	free(hdl);
}

static int
sio_tinyalsa_start(struct sio_hdl *sh)
{
	struct sio_tinyalsa_hdl *hdl = (struct sio_tinyalsa_hdl *)sh;

	DPRINTF("sio_tinyalsa_start:\n");

	hdl->ibpf = hdl->par.rchan * hdl->par.bps;
	hdl->obpf = hdl->par.pchan * hdl->par.bps;
	hdl->iused = 0;
	hdl->oused = 0;
	hdl->idelta = 0;
	hdl->odelta = 0;
	hdl->running = 0;


	if (hdl->sio.mode & SIO_PLAY) {
		if (pcm_prepare(hdl->opcm) < 0) {
			DTINYALSA("couldn't prepare play stream", hdl->opcm);
			hdl->sio.eof = 1;
			return 0;
		}
		hdl->otmpbuf = malloc(hdl->obpf);
		if (hdl->otmpbuf == NULL) {
			hdl->sio.eof = 1;
			return 0;
		}
		hdl->opartial = 0;
	}
	if (hdl->sio.mode & SIO_REC) {
		if (pcm_prepare(hdl->ipcm) < 0) {
			DTINYALSA("couldn't prepare play stream", hdl->opcm);
			hdl->sio.eof = 1;
			return 0;
		}
		hdl->itmpbuf = malloc(hdl->ibpf);
		if (hdl->itmpbuf == NULL) {
			hdl->sio.eof = 1;
			return 0;
		}
		hdl->ipartial = 0;
	}
	if ((hdl->sio.mode & SIO_PLAY) && (hdl->sio.mode & SIO_REC)) {
		if (pcm_link(hdl->ipcm, hdl->opcm) < 0) {
			DTINYALSA("couldn't link streams", hdl->ipcm);
			DTINYALSA("couldn't link streams", hdl->opcm);
			hdl->sio.eof = 1;
			return 0;
		}
	}
	if (!(hdl->sio.mode & SIO_PLAY)) {
		if (pcm_start(hdl->ipcm) < 0) {
			DTINYALSA("couldn't start rec stream", hdl->ipcm);
			hdl->sio.eof = 1;
			return 0;
		}
	}
	return 1;
}

static int
sio_tinyalsa_stop(struct sio_hdl *sh)
{
	struct sio_tinyalsa_hdl *hdl = (struct sio_tinyalsa_hdl *)sh;

	DPRINTF("sio_tinyalsa_stop:\n");

	if (hdl->sio.mode & SIO_PLAY) {
		if (pcm_stop(hdl->opcm) < 0) {
			DTINYALSA("couldn't stop play stream", hdl->opcm);
			hdl->sio.eof = 1;
			return 0;
		}
		free(hdl->otmpbuf);
	}
	if (hdl->sio.mode & SIO_REC) {
		if (pcm_stop(hdl->ipcm) < 0) {
			DTINYALSA("couldn't stop rec stream", hdl->ipcm);
			hdl->sio.eof = 1;
			return 0;
		}
		free(hdl->itmpbuf);
	}
	if ((hdl->sio.mode & SIO_PLAY) && (hdl->sio.mode & SIO_REC)) {
		if (pcm_unlink(hdl->ipcm) < 0) {
			DTINYALSA("couldn't unlink rec stream", hdl->ipcm);
			hdl->sio.eof = 1;
			return 0;
		}
		if (pcm_unlink(hdl->opcm) < 0) {
			DTINYALSA("couldn't unlink play streams", hdl->opcm);
			hdl->sio.eof = 1;
			return 0;
		}
	}
	DPRINTFN(2, "sio_tinyalsa_stop: stopped\n");
	return 1;
}

static int
sio_tinyalsa_setpar(struct sio_hdl *sh, struct sio_par *par)
{
	struct sio_tinyalsa_hdl *hdl = (struct sio_tinyalsa_hdl *)sh;
	struct pcm_config config, *oconfig;
	unsigned int round, bufsz;

	DPRINTF("sio_tinyalsa_setpar: bits=%d sig=%d le=%d round=%d pchan=%d rate=%d bufsz=%d appbufsz=%d\n",
	    par->bits, par->sig, par->le, par->round, par->pchan, par->rate,
	    par->bufsz, par->appbufsz);
	sio_alsa_enctofmt(hdl, &config.format, par->bits, par->sig, par->le);
	config.channels = par->pchan;

	if (par->round != ~0U && par->appbufsz != ~0U) {
		round = par->round;
		bufsz = par->appbufsz;
	} else if (par->round != ~0U) {
		round = par->round;
		bufsz = 2 * par->round;
	} else if (par->appbufsz != ~0U) {
		round = par->appbufsz / 2;
		bufsz = par->appbufsz;
	} else {
		/*
		 * even if it's not specified, we have to set the
		 * block size to ensure that both play and record
		 * direction get the same block size. Pick an
		 * arbitrary value that would work for most players at
		 * 48kHz, stereo, 16-bit.
		 */
		round = 512;
		bufsz = 1024;
	}

	config.period_size = round;
	config.period_count = bufsz / round;

	config.rate = par->rate;
	if (hdl->sio.mode & SIO_PLAY) {
		config.silence_threshold = 0;
		config.start_threshold = bufsz;
		config.stop_threshold = bufsz;
	}

	DPRINTF("sio_tinyalsa_setpar: round=%d bufsz=%d period_count=%d\n",
	    round, bufsz, bufsz / round);

	if (pcm_set_config(hdl->opcm, &config) < 0) {
		DTINYALSA("pcm_set_config", hdl->opcm);
		hdl->sio.eof = 1;
		return 0;
	}

	oconfig = pcm_get_config(hdl->opcm);
	if (oconfig == NULL) {
		DTINYALSA("pcm_get_config", hdl->opcm);
		hdl->sio.eof = 1;
		return 0;
	}

	sio_tinyalsa_fmttopar(hdl, oconfig->format, &hdl->par.bits,
	    &hdl->par.sig, &hdl->par.le);
	hdl->par.bufsz = oconfig->period_size * oconfig->period_count;
	hdl->par.appbufsz = hdl->par.bufsz;
	hdl->par.pchan = par->rchan = oconfig->channels;
	hdl->par.rate = oconfig->rate;
	hdl->par.round = oconfig->period_size;
	/* hdl->par.xrun = SIO_IGNORE; */
	hdl->par.msb = 1;
	hdl->par.bps = SIO_BPS(par->bits);

	return 1;
}

static int
sio_tinyalsa_getpar(struct sio_hdl *sh, struct sio_par *par)
{
	struct sio_tinyalsa_hdl *hdl = (struct sio_tinyalsa_hdl *)sh;
	*par = hdl->par;
	DPRINTF("sio_tinyalsa_getpar: bits=%d sig=%d le=%d round=%d pchan=%d rate=%d bufsz=%d appbufsz=%d\n",
	    par->bits, par->sig, par->le, par->round, par->pchan, par->rate,
	    par->bufsz, par->appbufsz);
	return 1;
}

static int
sio_tinyalsa_xrun(struct sio_tinyalsa_hdl *hdl)
{
	int clk;
	int wsil, rdrop, cmove;
	int rbpf, rround;
	int wbpf;

	DPRINTFN(2, "sio_tinyalsa_xrun:\n");
	if (_sndio_debug >= 2)
		_sio_printpos(&hdl->sio);

	/*
	 * we assume rused/wused are zero if rec/play modes are not
	 * selected. This allows us to keep the same formula for all
	 * modes, provided we set rbpf/wbpf to 1 to avoid division by
	 * zero.
	 *
	 * to understand the formula, draw a picture :)
	 */
	rbpf = (hdl->sio.mode & SIO_REC) ?
	    hdl->sio.par.bps * hdl->sio.par.rchan : 1;
	wbpf = (hdl->sio.mode & SIO_PLAY) ?
	    hdl->sio.par.bps * hdl->sio.par.pchan : 1;
	rround = hdl->sio.par.round * rbpf;

	clk = hdl->sio.cpos % hdl->sio.par.round;
	rdrop = (clk * rbpf - hdl->sio.rused) % rround;
	if (rdrop < 0)
		rdrop += rround;
	cmove = (rdrop + hdl->sio.rused) / rbpf;
	wsil = cmove * wbpf + hdl->sio.wused;

	DPRINTFN(2, "wsil = %d, cmove = %d, rdrop = %d\n", wsil, cmove, rdrop);

	if (!sio_tinyalsa_stop(&hdl->sio))
		return 0;
	if (!sio_tinyalsa_start(&hdl->sio))
		return 0;
	if (hdl->sio.mode & SIO_PLAY) {
		hdl->odelta -= cmove;
		hdl->sio.wsil = wsil;
	}
	if (hdl->sio.mode & SIO_REC) {
		hdl->idelta -= cmove;
		hdl->sio.rdrop = rdrop;
	}
	DPRINTFN(2, "xrun: corrected\n");
	DPRINTFN(2, "wsil = %d, rdrop = %d, odelta = %d, idelta = %d\n",
	    wsil, rdrop, hdl->odelta, hdl->idelta);
	return 1;
}


static size_t
sio_tinyalsa_read(struct sio_hdl *sh, void *buf, size_t len)
{
	struct sio_tinyalsa_hdl *hdl = (struct sio_tinyalsa_hdl *)sh;
	ssize_t n;

	DPRINTF("sio_tinyalsa_read:\n");

	while ((n = pcm_readi(hdl->opcm, buf, len)) < 0) {
		if (errno == EINTR)
			continue;
		if (errno != EAGAIN) {
			DPERROR("sio_tinyalsa_read: read");
			hdl->sio.eof = 1;
		}
		return 0;
	}
	if (n == 0) {
		DPRINTF("sio_tinyalsa_read: eof\n");
		hdl->sio.eof = 1;
		return 0;
	}
	return n;
}

static size_t
sio_tinyalsa_write(struct sio_hdl *sh, const void *buf, size_t len)
{
	struct sio_tinyalsa_hdl *hdl = (struct sio_tinyalsa_hdl *)sh;
	const unsigned char *data = buf;
	int n;
	unsigned int todo;

	if (len < hdl->obpf || hdl->opartial > 0) {
		todo = hdl->obpf - hdl->opartial;
		if (todo > 0) {
			if (todo > len)
				todo = len;
			memcpy(hdl->otmpbuf + hdl->opartial, buf, todo);
			hdl->opartial += todo;
			return todo;
		}
		len = hdl->obpf;
		buf = hdl->otmpbuf;
	}
	todo = len / hdl->obpf;
	if (todo == 0)
		return 0;

	DPRINTFN(5, "sio_tinyalsa_write: %u\n", todo);
	while ((n = pcm_writei(hdl->opcm, buf, todo)) < 0) {
		if (n == -EINTR)
			continue;
		if (n == -ESTRPIPE || n == -EPIPE) {
			sio_tinyalsa_xrun(hdl);
			return 0;
		}
		if (n != -EAGAIN) {
			DTINYALSA("couldn't write data", hdl->opcm);
			hdl->sio.eof = 1;
		}
		return 0;
	}
	hdl->odelta += n;
	if (buf == hdl->otmpbuf) {
		if (n > 0)
			hdl->opartial = 0;
		return 0;
	}
	return n * hdl->obpf;
}

void
sio_tinyalsa_onmove(struct sio_tinyalsa_hdl *hdl)
{
	int delta;
	DPRINTFN(5, "sio_tinyalsa_onmove:\n");

	if (hdl->running) {
		switch (hdl->sio.mode & (SIO_PLAY | SIO_REC)) {
		case SIO_PLAY:
			delta = hdl->odelta;
			break;
		case SIO_REC:
			delta = hdl->idelta;
			break;
		default: /* SIO_PLAY | SIO_REC */
			delta = hdl->odelta > hdl->idelta ?
				hdl->odelta : hdl->idelta;
		}
		if (delta <= 0)
			return;
	} else {
		delta = 0;
		hdl->running = 1;
	}
	DPRINTFN(5, "sio_tinyalsa_onmove: delta=%d\n", delta);
	_sio_onmove_cb(&hdl->sio, delta);
	if (hdl->sio.mode & SIO_PLAY)
		hdl->odelta -= delta;
	if (hdl->sio.mode & SIO_REC)
		hdl->idelta -= delta;
}

static int
sio_tinyalsa_nfds(struct sio_hdl *hdl)
{
	return 1;
}

static int
sio_tinyalsa_pollfd(struct sio_hdl *sh, struct pollfd *pfd, int events)
{
	struct sio_tinyalsa_hdl *hdl = (struct sio_tinyalsa_hdl *)sh;

	if (hdl->sio.eof)
		return 0;

#if 1
	hdl->events = events & (POLLIN | POLLOUT);
	if (!(hdl->sio.mode & SIO_PLAY))
		hdl->events &= ~POLLIN;
	if (!(hdl->sio.mode & SIO_REC))
		hdl->events &= ~POLLOUT;
#else
	hdl->events = events;
#endif
	if (!hdl->sio.started)
		hdl->events = 0;

	DPRINTFN(5, "sio_tinyalsa_pollfd: hdl->events=%03x SIO_PLAY=%d SIO_REC=%d\n", hdl->events, hdl->sio.mode & SIO_PLAY, hdl->sio.mode & SIO_REC);

	pfd->fd = hdl->ofd;
	pfd->events = events & POLLOUT ? POLLOUT : 0;

	if (!hdl->running && pcm_state(hdl->opcm) == PCM_STATE_RUNNING)
		sio_tinyalsa_onmove(hdl);
	/* sio_tinyalsa_onmove(hdl); */
	if (hdl->events & POLLOUT) {
		if (!hdl->running && pcm_state(hdl->opcm) == PCM_STATE_RUNNING)
			sio_tinyalsa_onmove(hdl);
	}
	if (hdl->events & POLLIN) {
		if (!hdl->running && pcm_state(hdl->opcm) == PCM_STATE_RUNNING)
			sio_tinyalsa_onmove(hdl);
	}

	return 1;
}

int
sio_tinyalsa_revents(struct sio_hdl *sh, struct pollfd *pfd)
{
	struct sio_tinyalsa_hdl *hdl = (struct sio_tinyalsa_hdl *)sh;
	long delay;
	int iused, oused;
	unsigned int oavail;
	int ostate;
	int dierr = 0, doerr = 0, offset, delta;
	int revents = pfd->revents;

	if ((pfd->revents & POLLHUP) ||
	    (pfd->revents & (POLLIN | POLLOUT)) == 0)
		return pfd->revents;

	/* DPRINTF("sio_tinyalsa_revents:\n"); */

	if (hdl->sio.eof)
		return POLLHUP;

#if 0
	if (hdl->events & POLLOUT) {
		DPRINTF("sio_tinyalsa_revents: POLLOUT\n");
	}
	if (hdl->events & POLLIN) {
		DPRINTF("sio_tinyalsa_revents: POLLIN\n");
	}
#endif

	delay = pcm_get_delay(hdl->opcm);
	if (delay < 0) /* this happens before the stream starts */
		delay = 0;

	if (hdl->sio.mode & SIO_PLAY) {
		ostate = pcm_state(hdl->opcm);
		if (ostate < 0)
			ostate = 0;
		if (ostate == PCM_STATE_XRUN) {
			if (!sio_tinyalsa_xrun(hdl))
				return POLLHUP;
			return 0;
		}
		DPRINTFN(5, "sio_tinyalsa_revents: state=%d\n", ostate);
		if (ostate == PCM_STATE_RUNNING ||
		    ostate == PCM_STATE_PREPARED) {
			oavail = pcm_avail_update(hdl->opcm);
			if (oavail < 0) {
				// XXX:
				hdl->sio.eof = 1;
				return POLLHUP;
			}
			oused = hdl->sio.par.bufsz - oavail;
			hdl->odelta -= oused - hdl->oused;
			hdl->oused = oused;
			DPRINTFN(5, "sio_tinyalsa_revents: oavail=%u oused=%d delay=%ld\n", oavail, oused, delay);
		}
	}

	/* if ((revents & (POLLIN | POLLOUT))) */
	if ((revents & (POLLIN | POLLOUT)) && hdl->running)
		sio_tinyalsa_onmove(hdl);
	return revents;
}
#endif /* defined USE_TINYALSA */
