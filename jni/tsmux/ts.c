#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

typedef struct pmt_map_t   /* Holds the mapping between the pmt-pid/pmt table */
{
    int i_pid;
    unsigned long i_prog;
} pmt_map_t;

typedef struct sdt_desc_t
{
    char *psz_provider;
    char *psz_service_name;  /* name of program */
} sdt_desc_t;

typedef struct
{
    int     i_depth;
    block_t *p_first;
    block_t **pp_last;
} sout_buffer_chain_t;

static inline void BufferChainInit  ( sout_buffer_chain_t *c )
{
    c->i_depth = 0;
    c->p_first = NULL;
    c->pp_last = &c->p_first;
}

static inline void BufferChainAppend( sout_buffer_chain_t *c, block_t *b )
{
    *c->pp_last = b;
    c->i_depth++;

    while( b->p_next )
    {
        b = b->p_next;
        c->i_depth++;
    }
    c->pp_last = &b->p_next;
}

static inline block_t *BufferChainGet( sout_buffer_chain_t *c )
{
    block_t *b = c->p_first;

    if( b )
    {
        c->i_depth--;
        c->p_first = b->p_next;

        if( c->p_first == NULL )
        {
            c->pp_last = &c->p_first;
        }

        b->p_next = NULL;
    }
    return b;
}

static inline block_t *BufferChainPeek( sout_buffer_chain_t *c )
{
    block_t *b = c->p_first;

    return b;
}

static inline void BufferChainClean( sout_buffer_chain_t *c )
{
    block_t *b;

    while( ( b = BufferChainGet( c ) ) )
    {
        block_Release( b );
    }
    BufferChainInit( c );
}

typedef struct ts_stream_t
{
    int             i_pid;
    vlc_fourcc_t    i_codec;

    int             i_stream_type;
    int             i_stream_id;
    int             i_continuity_counter;
    bool            b_discontinuity;

    /* to be used for carriege of DIV3 */
    vlc_fourcc_t    i_bih_codec;
    int             i_bih_width, i_bih_height;

    /* Specific to mpeg4 in mpeg2ts */
    int             i_es_id;

    int             i_decoder_specific_info;
    uint8_t         *p_decoder_specific_info;

    /* language is iso639-2T */
    int             i_langs;
    uint8_t         *lang;

    sout_buffer_chain_t chain_pes;
    mtime_t             i_pes_dts;
    mtime_t             i_pes_length;
    int                 i_pes_used;
    bool                b_key_frame;

} ts_stream_t;

struct sout_mux_sys_t
{
    int             i_pcr_pid;
    sout_input_t    *p_pcr_input;

    vlc_mutex_t     csa_lock;

    int             i_audio_bound;
    int             i_video_bound;

    bool            b_es_id_pid;
    bool            b_sdt;
    int             i_pid_video;
    int             i_pid_audio;
    int             i_pid_spu;
    int             i_pid_free; /* first usable pid */

    int             i_tsid;
    int             i_netid;
    int             i_num_pmt;
    int             i_pmtslots;
    int             i_pat_version_number;
    ts_stream_t     pat;

    int             i_pmt_version_number;
    ts_stream_t     pmt[MAX_PMT];
    pmt_map_t       pmtmap[MAX_PMT_PID];
    int             i_pmt_program_number[MAX_PMT];
    sdt_desc_t      sdt_descriptors[MAX_PMT];
    bool            b_data_alignment;

    int             i_mpeg4_streams;

    int             i_null_continuity_counter;  /* Needed ? */
    ts_stream_t     sdt;
    dvbpsi_pmt_t    *dvbpmt;

    /* for TS building */
    int64_t         i_bitrate_min;
    int64_t         i_bitrate_max;

    int64_t         i_shaping_delay;
    int64_t         i_pcr_delay;

    int64_t         i_dts_delay;

    bool            b_use_key_frames;

    mtime_t         i_pcr;  /* last PCR emited */

    csa_t           *csa;
    int             i_csa_pkt_size;
    bool            b_crypt_audio;
    bool            b_crypt_video;
};

