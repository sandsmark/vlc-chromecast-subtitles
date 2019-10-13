/*****************************************************************************
 * muxvtt.c: muxer for raw WEBVTT
 *****************************************************************************
 * Copyright (C) 2018 VideoLabs, VLC authors and VideoLAN
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston MA 02110-1301, USA.
 *****************************************************************************/
#ifdef HAVE_CONFIG_H
# include "config.h"
#endif

#include <vlc_common.h>
#include <vlc_plugin.h>
#include <vlc_codec.h>
#include <vlc_sout.h>
#include <vlc_memstream.h>

#include "../codec/webvtt/webvtt.h"
#include "../demux/mp4/minibox.h"

/*****************************************************************************
 * Exported prototypes
 *****************************************************************************/
static int Control( sout_mux_t *, int, va_list );
static int AddStream( sout_mux_t *, sout_input_t * );
static void DelStream( sout_mux_t *, sout_input_t * );
static int Mux      ( sout_mux_t * );

struct sout_mux_sys_t
{
    bool b_header_done;
    const sout_input_t *p_input;
};

static void OutputTime(struct vlc_memstream *ms, mtime_t i_time)
{
    mtime_t secs = i_time / CLOCK_FREQ;
    vlc_memstream_printf( ms, "%u:%02u:%02u.%03u",
                          (unsigned) (secs / 3600),
                          (unsigned) (secs % 3600 / 60),
                          (unsigned) (secs % 60),
                          (unsigned) (i_time % CLOCK_FREQ) / 1000 );
}

static block_t * UnpackISOBMFF( block_t *p_block )
{
    struct vlc_memstream ms;
    if( vlc_memstream_open( &ms ) )
        return NULL;

    mp4_box_iterator_t it;
    mp4_box_iterator_Init( &it, p_block->p_buffer, p_block->i_buffer );
    while( mp4_box_iterator_Next( &it ) )
    {
        if( it.i_type == ATOM_vttc || it.i_type == ATOM_vttx )
        {
            char *psz_iden = NULL;
            char *psz_sttg = NULL;
            char *psz_payl = NULL;

            mp4_box_iterator_t vtcc;
            mp4_box_iterator_Init( &vtcc, it.p_payload, it.i_payload );
            while( mp4_box_iterator_Next( &vtcc ) )
            {
                switch( vtcc.i_type )
                {
                    case ATOM_iden:
                        if(!psz_iden)
                            psz_iden = strndup( (const char*)vtcc.p_payload, vtcc.i_payload );
                        break;
                    case ATOM_sttg:
                        if(psz_sttg)
                        {
                            char *dup = strndup( (const char*)vtcc.p_payload, vtcc.i_payload );
                            if( dup )
                            {
                                char *psz;
                                if( asprintf( &psz, "%s %s", psz_sttg, dup ) >= 0 )
                                {
                                    free( psz_sttg );
                                    psz_sttg = psz;
                                }
                                free( dup );
                            }
                        }
                        else psz_sttg = strndup( (const char*)vtcc.p_payload, vtcc.i_payload );
                        break;
                    case ATOM_payl:
                        if(!psz_payl)
                            psz_payl = strndup( (const char*)vtcc.p_payload, vtcc.i_payload );
                        break;
                }
            }

			mtime_t start_time = p_block->i_dts - VLC_TICK_0;
            mtime_t end_time = p_block->i_dts - VLC_TICK_0 + p_block->i_length;
            
            if (start_time < 0)
				start_time = 0;
				
			if (end_time < 0)
				end_time = 0;
				
			if (end_time > start_time) {
            
				if( psz_iden )
					vlc_memstream_printf( &ms, "%s\n", psz_iden );
				
				OutputTime( &ms, start_time );
				vlc_memstream_printf( &ms, " --> " );
				OutputTime( &ms, end_time );

				if( psz_sttg )
					vlc_memstream_printf( &ms, " %s\n", psz_sttg );
				else
					vlc_memstream_putc( &ms, '\n' );

				vlc_memstream_printf( &ms, "%s\n\n", psz_payl );
			}

            free( psz_iden );
            free( psz_sttg );
            free( psz_payl );
        }
    }

    block_Release( p_block );

    if( vlc_memstream_close( &ms ) )
        return NULL;

    return block_heap_Alloc( ms.ptr, ms.length );
}

/*****************************************************************************
 * Open:
 *****************************************************************************/
int webvtt_OpenMuxer( vlc_object_t *p_this )
{
    sout_mux_t *p_mux = (sout_mux_t*)p_this;
    struct sout_mux_sys_t  *p_sys;

    p_mux->pf_control   = Control;
    p_mux->pf_addstream = AddStream;
    p_mux->pf_delstream = DelStream;
    p_mux->pf_mux       = Mux;

    p_mux->p_sys = p_sys = malloc( sizeof( struct sout_mux_sys_t ) );
    if( !p_sys )
        return VLC_ENOMEM;
    p_sys->b_header_done = false;
    p_sys->p_input = NULL;

    return VLC_SUCCESS;
}

/*****************************************************************************
 * Close:
 *****************************************************************************/

void webvtt_CloseMuxer( vlc_object_t * p_this )
{
    sout_mux_t *p_mux = (sout_mux_t*)p_this;
    struct sout_mux_sys_t *p_sys = p_mux->p_sys;

    free( p_sys );
}

static int Control( sout_mux_t *p_mux, int i_query, va_list args )
{
    VLC_UNUSED(p_mux);

    switch( i_query )
    {
        case MUX_CAN_ADD_STREAM_WHILE_MUXING:
            *(va_arg( args, bool * )) = false;
            return VLC_SUCCESS;

        case MUX_GET_ADD_STREAM_WAIT:
            *(va_arg( args, bool * )) = true;
            return VLC_SUCCESS;

        default:
            return VLC_EGENERIC;
   }
}

static int AddStream( sout_mux_t *p_mux, sout_input_t *p_input )
{
    struct sout_mux_sys_t *p_sys = p_mux->p_sys;
    if( p_input->fmt.i_codec != VLC_CODEC_WEBVTT ||
        p_sys->p_input )
        return VLC_EGENERIC;
    return VLC_SUCCESS;
}

static void DelStream( sout_mux_t *p_mux, sout_input_t *p_input )
{
    struct sout_mux_sys_t *p_sys = p_mux->p_sys;
    if( p_input == p_sys->p_input )
        p_sys->p_input = NULL;
}

static int Mux( sout_mux_t *p_mux )
{
    struct sout_mux_sys_t *p_sys = p_mux->p_sys;

    if( p_mux->i_nb_inputs == 0 )
        return VLC_SUCCESS;

    sout_input_t *p_input = p_mux->pp_inputs[0];
    block_fifo_t *p_fifo = p_input->p_fifo;

    if( !p_sys->b_header_done )
    {
        block_t *p_data = NULL;
        if( p_input->fmt.i_extra > 8 &&
           !memcmp( p_input->fmt.p_extra, "WEBVTT", 6 ) )
        {
            p_data = block_Alloc( p_input->fmt.i_extra + 2 );
            if( p_data )
            {
                memcpy( p_data->p_buffer, p_input->fmt.p_extra,
                                          p_input->fmt.i_extra );
                p_data->p_buffer[p_data->i_buffer - 2] = '\n';
                p_data->p_buffer[p_data->i_buffer - 1] = '\n';
            }
        }
        else
        {
            p_data = block_Alloc( 8 );
            if( p_data )
                memcpy( p_data->p_buffer, "WEBVTT\n\n", 8 );
        }

        if ( p_data )
            sout_AccessOutWrite( p_mux->p_access, p_data );
        p_sys->b_header_done = true;
    }

    for( size_t i=block_FifoCount( p_fifo ); i > 0; i-- )
    {
        block_t *p_data = block_FifoGet( p_fifo );
        p_data = UnpackISOBMFF( p_data );
        if( p_data )
            sout_AccessOutWrite( p_mux->p_access, p_data );
    }

    return VLC_SUCCESS;
}
