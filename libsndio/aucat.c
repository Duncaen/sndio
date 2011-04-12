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

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "aucat.h"
#ifdef COMPAT_STRLCPY
#include "bsd-compat.h"
#endif

int aucat_debug = 0;

#ifdef DEBUG
#define DPRINTF(...)						\
	do {							\
		if (aucat_debug > 0)				\
			fprintf(stderr, __VA_ARGS__);		\
	} while(0)
#define DPERROR(s)						\
	do {							\
		if (aucat_debug > 0)				\
			perror(s);				\
	} while(0)
#else
#define DPRINTF(...) do {} while(0)
#define DPERROR(s) do {} while(0)
#endif

/*
 * read a message, return 0 if not completed
 */
int
aucat_rmsg(struct aucat *hdl, int *eof)
{
	ssize_t n;
	unsigned char *data;

	if (hdl->rstate != RSTATE_MSG) {
		DPRINTF("aucat_rmsg: bad state\n");
		abort();
	}
	while (hdl->rtodo > 0) {
		data = (unsigned char *)&hdl->rmsg;
		data += sizeof(struct amsg) - hdl->rtodo;
		while ((n = read(hdl->fd, data, hdl->rtodo)) < 0) {
			if (errno == EINTR)
				continue;
			if (errno != EAGAIN) {
				*eof = 1;
				DPERROR("aucat_rmsg: read");
			}
			return 0;
		}
		if (n == 0) {
			DPRINTF("aucat_rmsg: eof\n");
			*eof = 1;
			return 0;
		}
		hdl->rtodo -= n;
	}
	if (hdl->rmsg.cmd == AMSG_DATA) {
		hdl->rtodo = hdl->rmsg.u.data.size;
		hdl->rstate = RSTATE_DATA;
	} else {
		hdl->rtodo = sizeof(struct amsg);
		hdl->rstate = RSTATE_MSG;
	}
	return 1;
}

/*
 * write a message, return 0 if not completed
 */
int
aucat_wmsg(struct aucat *hdl, int *eof)
{
	ssize_t n;
	unsigned char *data;

	if (hdl->wstate == WSTATE_IDLE)
		hdl->wstate = WSTATE_MSG;
		hdl->wtodo = sizeof(struct amsg);
	if (hdl->wstate != WSTATE_MSG) {
		DPRINTF("aucat_wmsg: bad state\n");
		abort();
	}
	while (hdl->wtodo > 0) {
		data = (unsigned char *)&hdl->wmsg;
		data += sizeof(struct amsg) - hdl->wtodo;
		while ((n = write(hdl->fd, data, hdl->wtodo)) < 0) {
			if (errno == EINTR)
				continue;
			if (errno != EAGAIN) {
				*eof = 1;
				DPERROR("aucat_wmsg: write");
			}
			return 0;
		}
		hdl->wtodo -= n;
	}
	if (hdl->wmsg.cmd == AMSG_DATA) {
		hdl->wtodo = hdl->wmsg.u.data.size;
		hdl->wstate = WSTATE_DATA;
	} else {
		hdl->wtodo = 0xdeadbeef;
		hdl->wstate = WSTATE_IDLE;
	}
	return 1;
}

size_t
aucat_rdata(struct aucat *hdl, void *buf, size_t len, int *eof)
{
	ssize_t n;

	if (hdl->rstate != RSTATE_DATA) {
		DPRINTF("aucat_rdata: bad state\n");
		abort();
	}
	if (len > hdl->rtodo)
		len = hdl->rtodo;
	while ((n = read(hdl->fd, buf, len)) < 0) {
		if (errno == EINTR)
			continue;
		if (errno != EAGAIN) {
			*eof = 1;
			DPERROR("aucat_rdata: read");
		}
		return 0;
	}
	if (n == 0) {
		DPRINTF("aucat_rdata: eof\n");
		*eof = 1;
		return 0;
	}
	hdl->rtodo -= n;
	if (hdl->rtodo == 0) {
		hdl->rstate = RSTATE_MSG;
		hdl->rtodo = sizeof(struct amsg);
	}
	DPRINTF("aucat_rdata: read: n = %zd\n", n);
	return n;
}

size_t
aucat_wdata(struct aucat *hdl, const void *buf, size_t len, unsigned wbpf, int *eof)
{
	ssize_t n;

	switch (hdl->wstate) {
	case WSTATE_IDLE:
		if (len > AMSG_DATAMAX)
			len = AMSG_DATAMAX;
		len -= len % wbpf;
		if (len == 0)
			len = wbpf;
		hdl->wmsg.cmd = AMSG_DATA;
		hdl->wmsg.u.data.size = len;
		hdl->wtodo = sizeof(struct amsg);
		hdl->wstate = WSTATE_MSG;
		/* FALLTHROUGH */
	case WSTATE_MSG:
		if (!aucat_wmsg(hdl, eof))
			return 0;
	}
	if (len > hdl->wtodo)
		len = hdl->wtodo;
	if (len == 0) {
		DPRINTF("aucat_wdata: len == 0\n");
		abort();
	}
	while ((n = write(hdl->fd, buf, len)) < 0) {
		if (errno == EINTR)
			continue;
		if (errno != EAGAIN) {
			*eof = 1;
			DPERROR("aucat_wdata: write");
		}
		return 0;
	}
	DPRINTF("aucat_wdata: write: n = %zd\n", n);
	hdl->wtodo -= n;
	if (hdl->wtodo == 0) {
		hdl->wstate = WSTATE_IDLE;
		hdl->wtodo = 0xdeadbeef;
	}
	return n;
}

int
aucat_open(struct aucat *hdl, const char *str, char *sock, unsigned mode, int nbio)
{
	extern char *__progname;
	int s, eof;
	char unit[4], *sep, *opt, *debug;
	struct sockaddr_un ca;
	socklen_t len = sizeof(struct sockaddr_un);
	uid_t uid;

	debug = getenv("AUCAT_DEBUG");
	if (debug)
		sscanf(debug, "%u", &aucat_debug);

	sep = strchr(str, '.');
	if (sep == NULL) {
		opt = "default";
		strlcpy(unit, str, sizeof(unit));
	} else {
		opt = sep + 1;
		if (sep - str >= sizeof(unit)) {
			DPRINTF("aucat_init: %s: too long\n", str);
			return NULL;
		}
		strlcpy(unit, str, opt - str);
	}
	DPRINTF("aucat_init: trying %s -> %s.%s\n", str, unit, opt);
	uid = geteuid();
	if (strchr(str, '/') != NULL)
		return NULL;
	snprintf(ca.sun_path, sizeof(ca.sun_path),
	    "/tmp/aucat-%u/%s%s", uid, sock, unit);
	ca.sun_family = AF_UNIX;

	s = socket(AF_UNIX, SOCK_STREAM, 0);
	if (s < 0)
		goto bad_free;
	while (connect(s, (struct sockaddr *)&ca, len) < 0) {
		if (errno == EINTR)
			continue;
		DPERROR("aucat_init: connect");
		/* try shared server */
		snprintf(ca.sun_path, sizeof(ca.sun_path),
		    "/tmp/aucat/%s%s", sock, unit);
		while (connect(s, (struct sockaddr *)&ca, len) < 0) {
			if (errno == EINTR)
				continue;
			DPERROR("aucat_init: connect");
			goto bad_connect;
		}
		break;
	}
	if (fcntl(s, F_SETFD, FD_CLOEXEC) < 0) {
		DPERROR("FD_CLOEXEC");
		goto bad_connect;
	}
	hdl->fd = s;
	hdl->rstate = RSTATE_MSG;
	hdl->rtodo = sizeof(struct amsg);
	hdl->wstate = WSTATE_IDLE;
	hdl->wtodo = 0xdeadbeef;

	/*
	 * say hello to server
	 */
	AMSG_INIT(&hdl->wmsg);
	hdl->wmsg.cmd = AMSG_HELLO;
	hdl->wmsg.u.hello.version = AMSG_VERSION;
	hdl->wmsg.u.hello.mode = mode;
	strlcpy(hdl->wmsg.u.hello.who, __progname,
	    sizeof(hdl->wmsg.u.hello.who));
	strlcpy(hdl->wmsg.u.hello.opt, opt,
	    sizeof(hdl->wmsg.u.hello.opt));
	hdl->wtodo = sizeof(struct amsg);
	if (!aucat_wmsg(hdl, &eof))
		goto bad_connect;
	hdl->rtodo = sizeof(struct amsg);
	if (!aucat_rmsg(hdl, &eof)) {
		DPRINTF("aucat_init: mode refused\n");
		goto bad_connect;
	}
	if (hdl->rmsg.cmd != AMSG_ACK) {
		DPRINTF("aucat_init: protocol err\n");
		goto bad_connect;
	}
	return 1;
 bad_connect:
	while (close(s) < 0 && errno == EINTR)
		; /* retry */
 bad_free:
	return 0;
}

void
aucat_close(struct aucat *hdl, int eof)
{
	char dummy[1];

	if (!eof) {
		AMSG_INIT(&hdl->wmsg);
		hdl->wmsg.cmd = AMSG_BYE;
		hdl->wtodo = sizeof(struct amsg);
		if (!aucat_wmsg(hdl, &eof))
			goto bad_close;
		while (read(hdl->fd, dummy, 1) < 0 && errno == EINTR)
			; /* nothing */
	}
 bad_close:
	while (close(hdl->fd) < 0 && errno == EINTR)
		; /* nothing */
}

int
aucat_setfl(struct aucat *hdl, int nbio, int *eof)
{
	if (fcntl(hdl->fd, F_SETFL, nbio ? O_NONBLOCK : 0) < 0) {
		DPERROR("aucat_setfl: fcntl");
		*eof = 1;
		return 0;
	}
	return 1;
}

int
aucat_pollfd(struct aucat *hdl, struct pollfd *pfd, int events)
{
	if (hdl->rstate == RSTATE_MSG)
		events |= POLLIN;
	pfd->fd = hdl->fd;
	pfd->events = events;
	return 1;
}

int
aucat_revents(struct aucat *hdl, struct pollfd *pfd)
{
	int revents = pfd->revents;

	DPRINTF("aucat_revents: revents: %x\n", revents);
	return revents;
}
