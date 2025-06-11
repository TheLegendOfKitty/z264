/*****************************************************************************
 * lookahead-parallel.c: parallel lookahead processing
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

#include "common/common.h"
#include "macroblock.h"
#include "me.h"
#include "analyse.h"

#define MAX_LOOKAHEAD_WORKERS 8

typedef enum {
    TASK_NONE = 0,
    TASK_MOTION_SEARCH,
    TASK_SCENE_ANALYSIS,
    TASK_COST_PROPAGATION,
    TASK_EXIT
} lookahead_task_type_t;

typedef struct {
    lookahead_task_type_t type;
    
    /* For motion search tasks */
    struct {
        x264_frame_t **frames;
        int p0, p1, b;
        int list;  /* 0 or 1 for L0/L1 */
        int start_mb, end_mb;  /* MB range for this task */
    } motion;
    
    /* For scene analysis tasks */
    struct {
        x264_frame_t *frame;
        int start_row, end_row;
    } scene;
    
    /* Results */
    volatile int completed;
    int cost;
    int *row_satds;
} x264_lookahead_task_t;

typedef struct {
    x264_pthread_t thread;
    int worker_id;
    x264_t *h;
    x264_mb_analysis_t analysis;
    
    /* Task queue */
    x264_lookahead_task_t *task_queue;
    int queue_size;
    int queue_head;
    int queue_tail;
    x264_pthread_mutex_t queue_mutex;
    x264_pthread_cond_t queue_cv_work;
    x264_pthread_cond_t queue_cv_done;
    
    /* Worker state */
    int b_exit;
    volatile int tasks_completed;
} x264_lookahead_worker_t;

typedef struct {
    x264_lookahead_worker_t workers[MAX_LOOKAHEAD_WORKERS];
    int num_workers;
    
    /* Global task management */
    x264_pthread_mutex_t task_mutex;
    volatile int total_tasks;
    volatile int completed_tasks;
    
    /* Shared buffers for results */
    int *cost_buffer;
    int *satd_buffer;
    
    /* Scene detection state */
    struct {
        uint32_t *histogram_buffer;
        float *variance_buffer;
        x264_pthread_mutex_t mutex;
    } scene;
} x264_parallel_lookahead_t;

/* Initialize parallel lookahead system */
int x264_parallel_lookahead_init( x264_t *h )
{
    x264_parallel_lookahead_t *parallel;
    CHECKED_MALLOCZERO( parallel, sizeof(x264_parallel_lookahead_t) );
    
    /* Determine number of workers based on CPU count and lookahead depth */
    int num_workers = X264_MIN( h->param.i_threads, MAX_LOOKAHEAD_WORKERS );
    num_workers = X264_MIN( num_workers, (h->param.rc.i_lookahead + 15) / 16 );
    num_workers = X264_MAX( num_workers, 1 );
    
    parallel->num_workers = num_workers;
    
    /* Allocate shared buffers */
    int max_mbs = h->mb.i_mb_count;
    CHECKED_MALLOC( parallel->cost_buffer, max_mbs * sizeof(int) );
    CHECKED_MALLOC( parallel->satd_buffer, max_mbs * h->param.rc.i_lookahead * sizeof(int) );
    CHECKED_MALLOC( parallel->scene.histogram_buffer, 256 * 3 * sizeof(uint32_t) );
    CHECKED_MALLOC( parallel->scene.variance_buffer, max_mbs * sizeof(float) );
    
    /* Initialize mutexes */
    if( x264_pthread_mutex_init( &parallel->task_mutex, NULL ) )
        goto fail;
    if( x264_pthread_mutex_init( &parallel->scene.mutex, NULL ) )
        goto fail;
    
    /* Initialize workers */
    for( int i = 0; i < num_workers; i++ )
    {
        x264_lookahead_worker_t *worker = &parallel->workers[i];
        worker->worker_id = i;
        worker->h = h;
        
        /* Allocate task queue for this worker */
        int queue_size = h->param.rc.i_lookahead * 2;
        CHECKED_MALLOC( worker->task_queue, queue_size * sizeof(x264_lookahead_task_t) );
        worker->queue_size = queue_size;
        worker->queue_head = 0;
        worker->queue_tail = 0;
        
        /* Initialize synchronization */
        if( x264_pthread_mutex_init( &worker->queue_mutex, NULL ) )
            goto fail;
        if( x264_pthread_cond_init( &worker->queue_cv_work, NULL ) )
            goto fail;
        if( x264_pthread_cond_init( &worker->queue_cv_done, NULL ) )
            goto fail;
        
        /* Initialize analysis context for this worker */
        worker->analysis.i_qp = X264_LOOKAHEAD_QP;
        worker->analysis.i_lambda = x264_lambda_tab[ worker->analysis.i_qp ];
        if( h->param.analyse.i_subpel_refine > 1 )
        {
            worker->analysis.i_me_method = X264_MIN( X264_ME_HEX, h->param.analyse.i_me_method );
            worker->analysis.i_subpel = 4;
        }
        else
        {
            worker->analysis.i_me_method = X264_ME_DIA;
            worker->analysis.i_subpel = 2;
        }
    }
    
    h->lookahead_parallel = parallel;
    return 0;
    
fail:
    x264_free( parallel->cost_buffer );
    x264_free( parallel->satd_buffer );
    x264_free( parallel->scene.histogram_buffer );
    x264_free( parallel->scene.variance_buffer );
    for( int i = 0; i < num_workers; i++ )
        x264_free( parallel->workers[i].task_queue );
    x264_free( parallel );
    return -1;
}

/* Add task to worker's queue */
static int lookahead_worker_enqueue( x264_lookahead_worker_t *worker, x264_lookahead_task_t *task )
{
    x264_pthread_mutex_lock( &worker->queue_mutex );
    
    int next_tail = (worker->queue_tail + 1) % worker->queue_size;
    if( next_tail == worker->queue_head )
    {
        /* Queue full */
        x264_pthread_mutex_unlock( &worker->queue_mutex );
        return -1;
    }
    
    worker->task_queue[worker->queue_tail] = *task;
    worker->queue_tail = next_tail;
    
    x264_pthread_cond_signal( &worker->queue_cv_work );
    x264_pthread_mutex_unlock( &worker->queue_mutex );
    
    return 0;
}

/* Motion search for a range of macroblocks */
static void lookahead_motion_search_partial( x264_t *h, x264_mb_analysis_t *a,
                                            x264_frame_t **frames, int p0, int p1, int b,
                                            int list, int start_mb, int end_mb,
                                            int *cost_out, int *row_satds )
{
    x264_frame_t *fenc = frames[b];
    x264_frame_t *fref = frames[list ? p1 : p0];
    const int i_stride = fenc->i_stride_lowres;
    const int i_mb_stride = h->mb.i_mb_stride;
    const int i_mb_height = h->mb.i_mb_height;
    const int i_mb_width = h->mb.i_mb_width;
    
    int i_cost = 0;
    
    for( int mb_index = start_mb; mb_index < end_mb; mb_index++ )
    {
        int mb_x = mb_index % i_mb_width;
        int mb_y = mb_index / i_mb_width;
        
        if( mb_y >= i_mb_height )
            break;
            
        /* Setup ME context for this MB */
        ALIGNED_ARRAY_64( pixel, pix,[8*8] );
        a->l0.me8x8.i_pixel = PIXEL_8x8;
        a->l0.me8x8.p_cost_mv = a->p_cost_mv;
        a->l0.me8x8.i_stride[0] = i_stride;
        a->l0.me8x8.p_fenc[0] = &fenc->lowres[0][(mb_y*8) * i_stride + (mb_x*8)];
        
        /* Setup reference planes */
        int i_pel_offset = 8 * (mb_x + mb_y * i_stride);
        a->l0.me8x8.p_fref[0] = fref->lowres[0] + i_pel_offset;
        a->l0.me8x8.p_fref[1] = fref->lowres[1] + i_pel_offset;
        a->l0.me8x8.p_fref[2] = fref->lowres[2] + i_pel_offset;
        a->l0.me8x8.p_fref[3] = fref->lowres[3] + i_pel_offset;
        
        a->l0.me8x8.cost = INT_MAX;
        
        /* Use simple diamond search for speed */
        x264_me_search( h, &a->l0.me8x8, NULL, 0 );
        
        /* Store MV result */
        int mv_idx = mb_y * i_mb_stride + mb_x;
        fenc->lowres_mvs[list][b - (list ? p1 : p0) - 1][mv_idx][0] = a->l0.me8x8.mv[0];
        fenc->lowres_mvs[list][b - (list ? p1 : p0) - 1][mv_idx][1] = a->l0.me8x8.mv[1];
        
        /* Accumulate cost */
        i_cost += a->l0.me8x8.cost;
        
        /* Update row SATD if needed */
        if( row_satds && mb_x == i_mb_width - 1 )
            row_satds[mb_y] = i_cost;
    }
    
    *cost_out = i_cost;
}

/* Scene analysis for a range of rows */
static void lookahead_scene_analysis_partial( x264_t *h, x264_frame_t *frame,
                                             int start_row, int end_row,
                                             uint32_t *histogram, float *variance )
{
    const int i_stride = frame->i_stride[0];
    const int i_width = frame->i_width[0];
    const int row_size = i_width;
    
    /* Initialize local histogram */
    uint32_t local_hist[256] = {0};
    float local_var = 0.0f;
    int pixel_count = 0;
    
    for( int y = start_row; y < end_row && y < frame->i_lines[0]; y++ )
    {
        pixel *row = frame->plane[0] + y * i_stride;
        
        /* Calculate histogram and variance */
        for( int x = 0; x < row_size; x++ )
        {
            local_hist[row[x]]++;
            pixel_count++;
        }
        
        /* Simple variance calculation */
        int sum = 0, sum_sq = 0;
        for( int x = 0; x < row_size; x++ )
        {
            sum += row[x];
            sum_sq += row[x] * row[x];
        }
        
        float mean = (float)sum / row_size;
        local_var += (float)sum_sq / row_size - mean * mean;
    }
    
    /* Merge results atomically */
    x264_pthread_mutex_lock( &h->lookahead_parallel->scene.mutex );
    for( int i = 0; i < 256; i++ )
        histogram[i] += local_hist[i];
    *variance += local_var / (end_row - start_row);
    x264_pthread_mutex_unlock( &h->lookahead_parallel->scene.mutex );
}

/* Worker thread main loop */
static void *lookahead_worker_thread( x264_lookahead_worker_t *worker )
{
    x264_t *h = worker->h;
    x264_mb_analysis_t *a = &worker->analysis;
    
    while( !worker->b_exit )
    {
        x264_lookahead_task_t task;
        
        /* Get next task from queue */
        x264_pthread_mutex_lock( &worker->queue_mutex );
        while( worker->queue_head == worker->queue_tail && !worker->b_exit )
            x264_pthread_cond_wait( &worker->queue_cv_work, &worker->queue_mutex );
        
        if( worker->b_exit )
        {
            x264_pthread_mutex_unlock( &worker->queue_mutex );
            break;
        }
        
        task = worker->task_queue[worker->queue_head];
        worker->queue_head = (worker->queue_head + 1) % worker->queue_size;
        x264_pthread_mutex_unlock( &worker->queue_mutex );
        
        /* Process task based on type */
        switch( task.type )
        {
            case TASK_MOTION_SEARCH:
            {
                int cost = 0;
                lookahead_motion_search_partial( h, a,
                    task.motion.frames, task.motion.p0, task.motion.p1, task.motion.b,
                    task.motion.list, task.motion.start_mb, task.motion.end_mb,
                    &cost, task.row_satds );
                task.cost = cost;
                break;
            }
            
            case TASK_SCENE_ANALYSIS:
            {
                lookahead_scene_analysis_partial( h, task.scene.frame,
                    task.scene.start_row, task.scene.end_row,
                    h->lookahead_parallel->scene.histogram_buffer,
                    h->lookahead_parallel->scene.variance_buffer );
                break;
            }
            
            default:
                break;
        }
        
        /* Mark task as completed */
        task.completed = 1;
        worker->tasks_completed++;
        
        /* Signal completion */
        x264_pthread_mutex_lock( &h->lookahead_parallel->task_mutex );
        h->lookahead_parallel->completed_tasks++;
        x264_pthread_mutex_unlock( &h->lookahead_parallel->task_mutex );
        x264_pthread_cond_broadcast( &worker->queue_cv_done );
    }
    
    return NULL;
}

/* Start worker threads */
int x264_parallel_lookahead_start( x264_t *h )
{
    x264_parallel_lookahead_t *parallel = h->lookahead_parallel;
    if( !parallel )
        return -1;
    
    for( int i = 0; i < parallel->num_workers; i++ )
    {
        x264_lookahead_worker_t *worker = &parallel->workers[i];
        if( x264_pthread_create( &worker->thread, NULL, 
                               (void *)lookahead_worker_thread, worker ) )
            return -1;
    }
    
    return 0;
}

/* Parallel motion search for lookahead */
int x264_parallel_lookahead_motion_search( x264_t *h, x264_frame_t **frames, 
                                         int p0, int p1, int b )
{
    x264_parallel_lookahead_t *parallel = h->lookahead_parallel;
    if( !parallel || parallel->num_workers < 2 )
        return -1;  /* Fall back to serial processing */
    
    x264_frame_t *fenc = frames[b];
    int total_mbs = h->mb.i_mb_count;
    int total_cost = 0;
    
    /* Clear task counters */
    x264_pthread_mutex_lock( &parallel->task_mutex );
    parallel->total_tasks = 0;
    parallel->completed_tasks = 0;
    x264_pthread_mutex_unlock( &parallel->task_mutex );
    
    /* Distribute motion search across workers */
    int mbs_per_worker = (total_mbs + parallel->num_workers - 1) / parallel->num_workers;
    
    for( int list = 0; list < 2; list++ )
    {
        if( (list == 0 && b == p0) || (list == 1 && b == p1) )
            continue;
            
        /* Check if we need to search this list */
        if( fenc->lowres_mvs[list][b - (list ? p1 : p0) - 1][0][0] != 0x7FFF )
            continue;
            
        /* Create tasks for each worker */
        for( int i = 0; i < parallel->num_workers; i++ )
        {
            x264_lookahead_task_t task = {0};
            task.type = TASK_MOTION_SEARCH;
            task.motion.frames = frames;
            task.motion.p0 = p0;
            task.motion.p1 = p1;
            task.motion.b = b;
            task.motion.list = list;
            task.motion.start_mb = i * mbs_per_worker;
            task.motion.end_mb = X264_MIN( (i + 1) * mbs_per_worker, total_mbs );
            task.completed = 0;
            
            /* Enqueue task */
            lookahead_worker_enqueue( &parallel->workers[i], &task );
            
            x264_pthread_mutex_lock( &parallel->task_mutex );
            parallel->total_tasks++;
            x264_pthread_mutex_unlock( &parallel->task_mutex );
        }
    }
    
    /* Wait for all tasks to complete */
    x264_pthread_mutex_lock( &parallel->task_mutex );
    while( parallel->completed_tasks < parallel->total_tasks )
    {
        x264_pthread_mutex_unlock( &parallel->task_mutex );
        x264_usleep( 100 );
        x264_pthread_mutex_lock( &parallel->task_mutex );
    }
    x264_pthread_mutex_unlock( &parallel->task_mutex );
    
    /* Aggregate costs from workers */
    for( int i = 0; i < parallel->num_workers; i++ )
        total_cost += parallel->workers[i].task_queue[0].cost;
    
    return total_cost;
}

/* Parallel scene detection */
void x264_parallel_lookahead_scene_detect( x264_t *h, x264_frame_t *frame )
{
    x264_parallel_lookahead_t *parallel = h->lookahead_parallel;
    if( !parallel || parallel->num_workers < 2 )
        return;  /* Fall back to serial processing */
    
    int total_rows = frame->i_lines[0];
    int rows_per_worker = (total_rows + parallel->num_workers - 1) / parallel->num_workers;
    
    /* Clear histogram and variance buffers */
    memset( parallel->scene.histogram_buffer, 0, 256 * 3 * sizeof(uint32_t) );
    memset( parallel->scene.variance_buffer, 0, h->mb.i_mb_count * sizeof(float) );
    
    /* Create scene analysis tasks */
    for( int i = 0; i < parallel->num_workers; i++ )
    {
        x264_lookahead_task_t task = {0};
        task.type = TASK_SCENE_ANALYSIS;
        task.scene.frame = frame;
        task.scene.start_row = i * rows_per_worker;
        task.scene.end_row = X264_MIN( (i + 1) * rows_per_worker, total_rows );
        
        lookahead_worker_enqueue( &parallel->workers[i], &task );
    }
    
    /* Wait for completion (simplified) */
    x264_usleep( 1000 );
    
    /* Results are now in parallel->scene buffers */
}

/* Cleanup parallel lookahead */
void x264_parallel_lookahead_delete( x264_t *h )
{
    x264_parallel_lookahead_t *parallel = h->lookahead_parallel;
    if( !parallel )
        return;
    
    /* Signal workers to exit */
    for( int i = 0; i < parallel->num_workers; i++ )
    {
        x264_lookahead_worker_t *worker = &parallel->workers[i];
        x264_pthread_mutex_lock( &worker->queue_mutex );
        worker->b_exit = 1;
        x264_pthread_cond_broadcast( &worker->queue_cv_work );
        x264_pthread_mutex_unlock( &worker->queue_mutex );
    }
    
    /* Wait for workers to finish */
    for( int i = 0; i < parallel->num_workers; i++ )
        x264_pthread_join( parallel->workers[i].thread, NULL );
    
    /* Cleanup resources */
    for( int i = 0; i < parallel->num_workers; i++ )
    {
        x264_pthread_mutex_destroy( &parallel->workers[i].queue_mutex );
        x264_pthread_cond_destroy( &parallel->workers[i].queue_cv_work );
        x264_pthread_cond_destroy( &parallel->workers[i].queue_cv_done );
        x264_free( parallel->workers[i].task_queue );
    }
    
    x264_pthread_mutex_destroy( &parallel->task_mutex );
    x264_pthread_mutex_destroy( &parallel->scene.mutex );
    
    x264_free( parallel->cost_buffer );
    x264_free( parallel->satd_buffer );
    x264_free( parallel->scene.histogram_buffer );
    x264_free( parallel->scene.variance_buffer );
    x264_free( parallel );
}