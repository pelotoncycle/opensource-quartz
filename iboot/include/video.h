/*
** MPC823 Video Controller
** =======================
** (C) 2000 by Paolo Scaffardi (arsenio@tin.it)
** AIRVENT SAM s.p.a - RIMINI(ITALY)
**
*/

#ifndef _VIDEO_H_
#define _VIDEO_H_

/* Video functions */

#if 0
int	video_init	(void *videobase);
#else
void video_clear(void);
void video_printf(const char *fmt, ...);
#endif
void	video_putc	(const char c);
void	video_puts	(const char *s);

#endif
