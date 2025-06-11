/*****************************************************************************
 * ref-adaptive.h: adaptive reference frame selection
 *****************************************************************************
 * Copyright (C) 2025 x264 project
 *
 * Authors: Claude AI Assistant
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

#ifndef X264_ENCODER_REF_ADAPTIVE_H
#define X264_ENCODER_REF_ADAPTIVE_H

/* Adaptive reference frame selection inspired by SVT-AV1 */

/* Reference frame quality score */
typedef struct
{
    float temporal_distance_factor;  /* Factor based on temporal distance */
    float quality_score;            /* Quality metric of reference frame */
    float motion_consistency;       /* Motion consistency with previous frames */
    float usage_history;           /* How often this ref was used recently */
    float combined_score;          /* Final weighted score */
} x264_ref_score_t;

/* Adaptive reference configuration */
typedef struct
{
    int enabled;                    /* Enable adaptive reference selection */
    int max_temporal_distance;     /* Maximum temporal distance to consider */
    float temporal_distance_weight; /* Weight for temporal distance factor */
    float quality_weight;          /* Weight for quality score */
    float motion_weight;           /* Weight for motion consistency */
    float usage_weight;            /* Weight for usage history */
    
    /* Dynamic reference limiting */
    int min_refs;                  /* Minimum references to always check */
    int max_refs;                  /* Maximum references to check */
    float score_threshold;         /* Threshold for reference inclusion */
    
    /* Content-adaptive parameters */
    float low_motion_factor;       /* Factor for low motion content */
    float high_motion_factor;      /* Factor for high motion content */
    
    /* Statistics for adaptation */
    int frame_count;
    float avg_motion_magnitude;
    float recent_ref_usage[16];    /* Usage history for recent frames */
} x264_adaptive_ref_t;

/* Function declarations */
void x264_adaptive_ref_init( x264_t *h );
void x264_adaptive_ref_update_scores( x264_t *h, int list );
int x264_adaptive_ref_filter_list( x264_t *h, int list, int *ref_list, int max_refs );
void x264_adaptive_ref_update_usage( x264_t *h, int list, int ref_idx );
void x264_adaptive_ref_update_motion_stats( x264_t *h, int16_t mv[2] );

/* Inline utility functions */
static inline float x264_adaptive_ref_temporal_factor( int distance, int max_distance )
{
    if( distance <= 0 || max_distance <= 0 )
        return 1.0f;
    
    /* Exponential decay based on temporal distance */
    return expf( -((float)distance / max_distance) * 2.0f );
}

static inline float x264_adaptive_ref_motion_factor( float motion_magnitude )
{
    /* Higher motion needs more references */
    return 1.0f + (motion_magnitude / 100.0f) * 0.5f;
}

#endif