/*****************************************************************************
 * ref-adaptive.c: adaptive reference frame selection
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

#include "common/common.h"
#include "ref-adaptive.h"
#include <math.h>

/****************************************************************************
 * x264_adaptive_ref_init:
 * Initialize adaptive reference frame selection
 ****************************************************************************/
void x264_adaptive_ref_init( x264_t *h )
{
    
    if( !h->param.analyse.b_adaptive_ref )
    {
        h->adaptive_ref.enabled = 0;
        return;
    }
    
    h->adaptive_ref.enabled = 1;
    
    /* Set default parameters based on content complexity */
    h->adaptive_ref.max_temporal_distance = h->param.analyse.i_adaptive_ref_temporal_distance;
    h->adaptive_ref.temporal_distance_weight = 0.4f;
    h->adaptive_ref.quality_weight = 0.3f;
    h->adaptive_ref.motion_weight = 0.2f;
    h->adaptive_ref.usage_weight = 0.1f;
    
    /* Dynamic limiting parameters */
    h->adaptive_ref.min_refs = X264_MAX( 1, h->param.i_frame_reference / 3 );
    h->adaptive_ref.max_refs = h->param.i_frame_reference;
    h->adaptive_ref.score_threshold = h->param.analyse.f_adaptive_ref_threshold;
    
    /* Content-adaptive factors */
    h->adaptive_ref.low_motion_factor = 0.7f;   /* Reduce refs for low motion */
    h->adaptive_ref.high_motion_factor = 1.3f;  /* Increase refs for high motion */
    
    /* Initialize statistics */
    h->adaptive_ref.frame_count = 0;
    h->adaptive_ref.avg_motion_magnitude = 0.0f;
    memset( h->adaptive_ref.recent_ref_usage, 0, sizeof(h->adaptive_ref.recent_ref_usage) );
    
    x264_log( h, X264_LOG_INFO, "adaptive reference frame selection enabled: max_temporal_distance %d, threshold %.2f\n",
              h->adaptive_ref.max_temporal_distance, h->adaptive_ref.score_threshold );
}

/****************************************************************************
 * x264_adaptive_ref_update_scores:
 * Update quality scores for reference frames
 ****************************************************************************/
void x264_adaptive_ref_update_scores( x264_t *h, int list )
{
    if( !h->adaptive_ref.enabled )
        return;
    
    int num_refs = h->i_ref[list];
    if( num_refs <= 1 )
        return;
    
    for( int i = 0; i < num_refs; i++ )
    {
        x264_frame_t *ref_frame = h->fref[list][i];
        /* Use direct struct access since ref_score is an anonymous struct */
        
        /* Calculate temporal distance factor */
        int temporal_distance = h->fenc->i_frame - ref_frame->i_frame;
        ref_frame->ref_score.temporal_distance_factor = x264_adaptive_ref_temporal_factor( 
            temporal_distance, h->adaptive_ref.max_temporal_distance );
        
        /* Calculate quality score based on frame QP and type */
        float base_quality = 100.0f / (1.0f + ref_frame->f_qp_avg_rc);
        if( ref_frame->i_type == X264_TYPE_IDR || ref_frame->i_type == X264_TYPE_I )
            base_quality *= 1.2f;  /* I-frames are higher quality references */
        else if( ref_frame->i_type == X264_TYPE_B )
            base_quality *= 0.8f;  /* B-frames are lower quality references */
        
        ref_frame->ref_score.quality_score = base_quality;
        
        /* Calculate motion consistency (simple version) */
        if( i > 0 && h->adaptive_ref.frame_count > 0 )
        {
            /* Use motion magnitude as a proxy for consistency */
            ref_frame->ref_score.motion_consistency = 1.0f / (1.0f + h->adaptive_ref.avg_motion_magnitude / 10.0f);
        }
        else
        {
            ref_frame->ref_score.motion_consistency = 1.0f;
        }
        
        /* Update usage history */
        ref_frame->ref_score.usage_history = h->adaptive_ref.recent_ref_usage[X264_MIN(i, 15)];
        
        /* Calculate combined score */
        ref_frame->ref_score.combined_score = 
            h->adaptive_ref.temporal_distance_weight * ref_frame->ref_score.temporal_distance_factor +
            h->adaptive_ref.quality_weight * (ref_frame->ref_score.quality_score / 100.0f) +
            h->adaptive_ref.motion_weight * ref_frame->ref_score.motion_consistency +
            h->adaptive_ref.usage_weight * ref_frame->ref_score.usage_history;
    }
}

/****************************************************************************
 * x264_adaptive_ref_filter_list:
 * Filter reference list based on adaptive scoring
 ****************************************************************************/
int x264_adaptive_ref_filter_list( x264_t *h, int list, int *ref_list, int max_refs )
{
    if( !h->adaptive_ref.enabled )
    {
        /* Default behavior - use all available references */
        for( int i = 0; i < max_refs; i++ )
            ref_list[i] = i;
        return max_refs;
    }
    
    int num_refs = h->i_ref[list];
    if( num_refs <= h->adaptive_ref.min_refs )
    {
        /* Too few refs, use all */
        for( int i = 0; i < num_refs; i++ )
            ref_list[i] = i;
        return num_refs;
    }
    
    /* Content-adaptive reference count */
    float motion_factor = x264_adaptive_ref_motion_factor( h->adaptive_ref.avg_motion_magnitude );
    int target_refs = (int)(h->adaptive_ref.min_refs + 
                          (h->adaptive_ref.max_refs - h->adaptive_ref.min_refs) * motion_factor * 0.5f);
    target_refs = x264_clip3( target_refs, h->adaptive_ref.min_refs, h->adaptive_ref.max_refs );
    target_refs = X264_MIN( target_refs, max_refs );
    
    /* Create list of references with scores */
    typedef struct {
        int ref_idx;
        float score;
    } ref_candidate_t;
    
    ref_candidate_t candidates[16];
    int num_candidates = 0;
    
    for( int i = 0; i < num_refs && i < 16; i++ )
    {
        x264_frame_t *ref_frame = h->fref[list][i];
        float score = ref_frame->ref_score.combined_score;
        
        /* Always include reference 0 and high-scoring references */
        if( i == 0 || score >= h->adaptive_ref.score_threshold )
        {
            candidates[num_candidates].ref_idx = i;
            candidates[num_candidates].score = score;
            num_candidates++;
        }
    }
    
    /* Sort candidates by score (descending) */
    for( int i = 0; i < num_candidates - 1; i++ )
    {
        for( int j = i + 1; j < num_candidates; j++ )
        {
            if( candidates[j].score > candidates[i].score )
            {
                ref_candidate_t temp = candidates[i];
                candidates[i] = candidates[j];
                candidates[j] = temp;
            }
        }
    }
    
    /* Select top references up to target count */
    int selected_refs = X264_MIN( num_candidates, target_refs );
    for( int i = 0; i < selected_refs; i++ )
    {
        ref_list[i] = candidates[i].ref_idx;
    }
    
    return selected_refs;
}

/****************************************************************************
 * x264_adaptive_ref_update_usage:
 * Update reference frame usage statistics
 ****************************************************************************/
void x264_adaptive_ref_update_usage( x264_t *h, int list, int ref_idx )
{
    
    if( !h->adaptive_ref.enabled || ref_idx >= 16 )
        return;
    
    /* Update usage history with exponential smoothing */
    float alpha = 0.1f;  /* Smoothing factor */
    h->adaptive_ref.recent_ref_usage[ref_idx] = alpha + (1.0f - alpha) * h->adaptive_ref.recent_ref_usage[ref_idx];
    
    /* Decay other reference usage */
    for( int i = 0; i < 16; i++ )
    {
        if( i != ref_idx )
            h->adaptive_ref.recent_ref_usage[i] *= (1.0f - alpha * 0.1f);
    }
}

/****************************************************************************
 * x264_adaptive_ref_update_motion_stats:
 * Update motion statistics for adaptive decisions
 ****************************************************************************/
void x264_adaptive_ref_update_motion_stats( x264_t *h, int16_t mv[2] )
{
    if( !h->adaptive_ref.enabled )
        return;
    
    /* Calculate motion magnitude */
    float motion_magnitude = sqrtf( mv[0] * mv[0] + mv[1] * mv[1] );
    
    /* Update average with exponential smoothing */
    float alpha = 0.05f;  /* Slow adaptation to avoid rapid changes */
    if( h->adaptive_ref.frame_count == 0 )
    {
        h->adaptive_ref.avg_motion_magnitude = motion_magnitude;
    }
    else
    {
        h->adaptive_ref.avg_motion_magnitude = alpha * motion_magnitude + 
                                  (1.0f - alpha) * h->adaptive_ref.avg_motion_magnitude;
    }
    
    h->adaptive_ref.frame_count++;
}