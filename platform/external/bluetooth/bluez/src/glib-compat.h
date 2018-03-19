/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2004-2011  Marcel Holtmann <marcel@holtmann.org>
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

static inline void g_slist_free_full(GSList *list, GDestroyNotify free_func)
{
	g_slist_foreach(list, (GFunc) free_func, NULL);
	g_slist_free(list);
}

#ifdef ANDROID
#include <time.h>

struct _GCompatTimer {
	struct timespec ts;
};
typedef struct _GCompatTimer GCompatTimer;


static inline void g_compat_timer_start(GCompatTimer *timer)
{
	clock_gettime(CLOCK_MONOTONIC, &timer->ts);
}

static inline GCompatTimer *g_compat_timer_new()
{
	GCompatTimer *timer = g_new(GCompatTimer, 1);
	g_compat_timer_start(timer);
	return timer;
}

static inline gdouble g_compat_timer_elapsed(GCompatTimer *timer, gulong *microseconds)
{
	struct timespec ts;
	gdouble ret;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return  (gdouble)(ts.tv_sec - timer->ts.tv_sec) +
	         ((gdouble)(ts.tv_nsec - timer->ts.tv_nsec) / 1000000000.0);
}

static inline void g_compat_timer_destroy(GCompatTimer *timer)
{
	g_free(timer);
}
#endif

#if defined(NEED_G_LIST_FREE_FULL) || defined(ANDROID)
static inline void g_list_free_full(GList *list, GDestroyNotify free_func)
{
	g_list_foreach(list, (GFunc) free_func, NULL);
	g_list_free(list);
}
#endif
