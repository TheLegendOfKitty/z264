/*****************************************************************************
 * lookahead-parallel.h: parallel lookahead processing header
 *****************************************************************************
 * Copyright (C) 2025 x264 project
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02111, USA.
 *
 * This program is also available under a commercial proprietary license.
 * For more information, contact us at licensing@x264.com.
 *****************************************************************************/

#ifndef X264_LOOKAHEAD_PARALLEL_H
#define X264_LOOKAHEAD_PARALLEL_H

/* Forward declaration */
typedef struct x264_parallel_lookahead_t x264_parallel_lookahead_t;

/* Initialize parallel lookahead system */
int x264_parallel_lookahead_init( x264_t *h );

/* Start worker threads */
int x264_parallel_lookahead_start( x264_t *h );

/* Parallel motion search for lookahead frames
 * Returns total cost or -1 if parallel processing not available */
int x264_parallel_lookahead_motion_search( x264_t *h, x264_frame_t **frames, 
                                         int p0, int p1, int b );

/* Parallel scene detection and analysis */
void x264_parallel_lookahead_scene_detect( x264_t *h, x264_frame_t *frame );

/* Cleanup parallel lookahead resources */
void x264_parallel_lookahead_delete( x264_t *h );

#endif /* X264_LOOKAHEAD_PARALLEL_H */