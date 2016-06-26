#include <jni.h>
#include <android/log.h>
#include <stdint.h>
#include <pthread.h>

#include <netdb.h>
#include <time.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <limits.h>
#include <fcntl.h>
#include <errno.h>

#include "tsmux/tsmux.h"
#include "tsmux/vlc_block.h"

#include "x264/x264.h"

// Slice type define
#define H264_SLICE_TYPE_IDR           0x0001
#define H264_SLICE_TYPE_I             0x0002
#define H264_SLICE_TYPE_P             0x0003
#define H264_SLICE_TYPE_BREF          0x0004  /* Non-disposable B-frame */
#define H264_SLICE_TYPE_B             0x0005

#define TYPE_H264_VIDEO 0x0000
#define TYPE_FAAC_AUDIO 0x0001

#define IS_H264_SLICE_TYPE_I(x) ((x)==H264_SLICE_TYPE_I || (x)==H264_SLICE_TYPE_IDR)
#define IS_H264_SLICE_TYPE_B(x) ((x)==H264_SLICE_TYPE_B || (x)==H264_SLICE_TYPE_BREF)

#ifndef LOG_TAG
#define  LOG_TAG    "libjxcodec"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#endif


#define MAX_OUTPUT_BUFFER 100000
#define TS_SENDER_BUFFER_SIZE 188 * 7 

#define USE_YUV_NV12 1

typedef struct sout_t sout_t;

typedef struct video_encoder_t
{
	x264_t * handle;
	x264_param_t * param;
	x264_picture_t *picture;
	x264_nal_t *p_nals;	
	mtime_t i_dts;	
	float f_fps;
	es_format_t fmt_out;
	sout_input_t * p_in;
	sout_t * p_sout;	
	unsigned char * p_pic_y;
	unsigned char * p_pic_uv;
}video_encoder_t;

#define MAX_RTP_TS_PACKET_SIZE ( 7 * 188 + 12 )
typedef struct rtp_buffer_t
{
  unsigned char p_buffer[MAX_RTP_TS_PACKET_SIZE];
  unsigned short i_buffer;
}rtp_buffer_t;

typedef struct sout_t
{
	int fd_udp;
	bool b_closed;
	pthread_mutex_t lock;
	pthread_t thread;
	pthread_t thread_mux;
	video_encoder_t *p_video_enc;
	block_fifo_t * p_fifo;
	block_fifo_t * p_es_fifo;
	sout_input_t *p_video_input;
	sout_input_t *p_audio_input;
	
	es_format_t fmt_video_in;
	es_format_t fmt_audio_in;
	mtime_t i_audio_pts;
	mtime_t i_video_pts;
	mtime_t i_audio_pts_increment;
	mtime_t i_video_pts_increment;

	/**
	 * rtp sender  
	*/
	unsigned long  i_rtp_ssrc;
	unsigned short i_rtp_last_seq;
	rtp_buffer_t   p_rtp_buffer;
	mtime_t i_dts;	
	
	sout_mux_t * p_mux;	
};




static inline
unsigned long mdate(){
	struct timeval tv;
	gettimeofday(&tv,0);
	return tv.tv_sec*1000+tv.tv_usec/1000;
}

static void h264_ts_callback(void * p_private, unsigned char * p_ts_data , size_t i_size )
{
	sout_t * p_sout = (sout_t *) p_private;

	int i_buffer = i_size;
	unsigned char * pbuffer = p_ts_data;	
	
	while(i_buffer)
	{
		int i_copy = __MIN ( MAX_RTP_TS_PACKET_SIZE - p_sout->p_rtp_buffer.i_buffer , i_buffer );
		memcpy( (p_sout->p_rtp_buffer.p_buffer + p_sout->p_rtp_buffer.i_buffer), pbuffer , i_copy );
		//printf("buffer %d , copy %d \n",i_buffer,i_copy);
		uint32_t i_timestamp = p_sout->i_dts;
		p_sout->p_rtp_buffer.p_buffer[0] = 0x80;
		p_sout->p_rtp_buffer.p_buffer[1] = 0x80|33;
		p_sout->p_rtp_buffer.p_buffer[2] = ( p_sout->i_rtp_last_seq >> 8)&0xff;
		p_sout->p_rtp_buffer.p_buffer[3] = (  p_sout->i_rtp_last_seq     )&0xff;
		p_sout->p_rtp_buffer.p_buffer[4] = ( i_timestamp >> 24 )&0xff;
		p_sout->p_rtp_buffer.p_buffer[5] = ( i_timestamp >> 16 )&0xff;
		p_sout->p_rtp_buffer.p_buffer[6] = ( i_timestamp >>  8 )&0xff;
		p_sout->p_rtp_buffer.p_buffer[7] = ( i_timestamp       )&0xff;
		memcpy( p_sout->p_rtp_buffer.p_buffer + 8, (unsigned char *)&p_sout->i_rtp_ssrc, 4 );
  
		p_sout->p_rtp_buffer.i_buffer += i_copy;
		pbuffer += i_copy;
		i_buffer -= i_copy;
		if ( p_sout->p_rtp_buffer.i_buffer == MAX_RTP_TS_PACKET_SIZE )
		{
				/* Add a file descriptor to the set */
		
			block_t * p_ts = block_Alloc( p_sout->p_rtp_buffer.i_buffer );
			memcpy( p_ts->p_buffer,p_sout->p_rtp_buffer.p_buffer
					,p_sout->p_rtp_buffer.i_buffer);
			p_ts->i_buffer = p_sout->p_rtp_buffer.i_buffer;
			block_FifoPut(p_sout->p_fifo,p_ts);	
	
			p_sout->p_rtp_buffer.i_buffer = 12;
			p_sout->i_rtp_last_seq++;
			//LOGI("Jxstreaming send to %d",bytes );
		}
	}		
	
}

static int wait_for_send( int sockfd, fd_set * wfds , struct timeval *tv ){
	
	
	int s_rc;
    while ((s_rc = select(sockfd+1, NULL, wfds, NULL, tv)) == -1) {
        if (errno == EINTR) {
            /* Necessary after an error */
            FD_ZERO(wfds);
            FD_SET(sockfd, wfds);
        } else {
            return -1;
        }
    }

    if (s_rc == 0) {
        /* Timeout */
        errno = ETIMEDOUT;
        return -1;
    }
    return s_rc;
}

void mux_thread( void *arg)
{
	sout_t * p_sout = (sout_t*)arg;
	bool closed = false;

	pthread_mutex_lock(&p_sout->lock);
	closed = p_sout->b_closed;
	pthread_mutex_unlock(&p_sout->lock);
	
	LOGI("Jxstreaming mux thread started");
	while(!closed)
	{	

		block_t *p_block_in = block_FifoGet(p_sout->p_es_fifo);	
		
		if ( p_block_in && !(p_block_in->i_flags & BLOCK_FLAG_PREROLL) )
		{
			sout_block_mux(p_block_in->p_owner,p_block_in);
		}else if ( p_block_in ) block_Release(p_block_in);
		
		pthread_mutex_lock(&p_sout->lock);
		closed = p_sout->b_closed;
		pthread_mutex_unlock(&p_sout->lock);
	}
	LOGI("Jxstreaming mux thread ended");
}

void send_thread( void * arg)
{	
	sout_t * p_sout = (sout_t*)arg;
	bool closed = false;
	uint16_t i_pos = 12;
	unsigned char send_buffer[MAX_RTP_TS_PACKET_SIZE];
	unsigned long  rtp_ssrc_ = rand();
	unsigned short rtp_last_seq_ = rand();
	int timeout = 1000;
    struct timeval tv;
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = ( timeout % 1000 ) * 1000;


	pthread_mutex_lock(&p_sout->lock);
	closed = p_sout->b_closed;
	pthread_mutex_unlock(&p_sout->lock);
	
	LOGI("Jxstreaming send thread started");
	while(!closed)
	{	
		block_t *p_block_in = block_FifoGet(p_sout->p_fifo);		
		if ( p_block_in && !(p_block_in->i_flags & BLOCK_FLAG_PREROLL) )
		{
			int i_buffer = p_block_in->i_buffer;
			unsigned char * pbuffer = p_block_in->p_buffer;
			#if 0
			while(i_buffer)
			{
				int i_copy = __MIN ( MAX_RTP_TS_PACKET_SIZE - i_pos , i_buffer );
				memcpy( (send_buffer + i_pos), pbuffer , i_copy );
				//printf("buffer %d , copy %d \n",i_buffer,i_copy);
				uint32_t i_timestamp = mdate()/90;
				send_buffer[0] = 0x80;
				send_buffer[1] = 0x80|33;
				send_buffer[2] = ( rtp_last_seq_ >> 8)&0xff;
				send_buffer[3] = ( rtp_last_seq_     )&0xff;
				send_buffer[4] = ( i_timestamp >> 24 )&0xff;
				send_buffer[5] = ( i_timestamp >> 16 )&0xff;
				send_buffer[6] = ( i_timestamp >>  8 )&0xff;
				send_buffer[7] = ( i_timestamp       )&0xff;
				memcpy( send_buffer + 8, (unsigned char *)&rtp_ssrc_, 4 );
		  
				i_pos += i_copy;
				pbuffer += i_copy;
				i_buffer -= i_copy;
				if ( i_pos == MAX_RTP_TS_PACKET_SIZE )
				{
					    /* Add a file descriptor to the set */
					fd_set wfds;
					FD_ZERO(&wfds);
					FD_SET(p_sout->fd_udp, &wfds);
					wait_for_send(p_sout->fd_udp, &wfds,&tv); 
					int bytes = send(p_sout->fd_udp,send_buffer,i_pos,0); 
					i_pos = 12;
					//LOGI("Jxstreaming send to %d",bytes );
					rtp_last_seq_++;
				}
			}	
			#else
			int bytes = send(p_sout->fd_udp,pbuffer,i_buffer,0);
			LOGI("Jxstreaming send to %d",bytes );
			rtp_last_seq_++;
			#endif
			
		}
		block_Release(p_block_in);			
		pthread_mutex_lock(&p_sout->lock);
		closed = p_sout->b_closed;
		pthread_mutex_unlock(&p_sout->lock);
	}
	LOGI("Jxstreaming send thread ended");
}

jlong 
Java_com_smartvision_jxvideoh264_Jxstreaming_create( JNIEnv *env,
												jobject this,
												jstring server,
												jint port )
{
	LOGI("Jxstreaming create start");
	int opt = 1;
	int len = sizeof(opt);

	sout_t* p_sys  = NULL;
	const char * addr = NULL;
	struct sockaddr_in addr_server;
	socklen_t addr_len = sizeof(struct sockaddr_in);
	p_sys = malloc( sizeof (sout_t));

	if ( p_sys == NULL )
		return 0;

	p_sys->p_fifo = block_FifoNew();
	p_sys->p_es_fifo = block_FifoNew();
	
	p_sys->p_mux = soutOpen(NULL,h264_ts_callback,p_sys);
	p_sys->p_video_enc = NULL;
	
	p_sys->p_video_input = NULL;
	p_sys->p_audio_input = NULL;
	p_sys->i_audio_pts = VLC_TS_0;
	p_sys->i_video_pts = VLC_TS_0;
	p_sys->i_video_pts_increment = 0;
	p_sys->i_audio_pts_increment = 0;
	
	/**
	 * rtp setting 
	 **/
	p_sys->i_rtp_ssrc = VLC_CODEC_H264;
	p_sys->i_rtp_last_seq = rand();
	p_sys->p_rtp_buffer.i_buffer = 12;
	p_sys->i_dts = VLC_TS_0;
	
	addr = (*env)->GetStringUTFChars(env, server, NULL);	
	bzero(&addr_server,sizeof(addr_server));
	addr_server.sin_family=AF_INET;
	addr_server.sin_addr.s_addr=inet_addr(addr);
	addr_server.sin_port=htons(port);	
	(*env)->ReleaseStringUTFChars(env,server, addr);
	// create udp socket 
	p_sys->fd_udp = socket ( AF_INET,SOCK_DGRAM,0);
	
	if(connect(p_sys->fd_udp,(struct sockaddr*)&addr_server,sizeof(addr_server))==-1)
	{
		LOGI("error in connecting %d",errno );
		close(p_sys->fd_udp);
	}	
	
	opt = 64 * 1024 ;
	setsockopt(p_sys->fd_udp,SOL_SOCKET,SO_SNDBUF,(char*)&opt,len);

	p_sys->b_closed = false;
	pthread_mutex_init(&p_sys->lock,NULL);
	
	pthread_create(&p_sys->thread,0,send_thread,(void*)p_sys);
	pthread_create(&p_sys->thread_mux,0,mux_thread,(void*)p_sys);
	LOGI("Jxstreaming create End");
	
	return (jlong)p_sys;
}

static const int pi_sample_rates[16] =
{
    96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050,
    16000, 12000, 11025, 8000,  7350,  0,     0,     0
};

jlong
Java_com_smartvision_jxvideoh264_Jxstreaming_addStream(JNIEnv *env,
														jobject this,
														jlong handle,
														jint type,
														jint frame_rate,
														jbyteArray extra,
														jint i_extra )
{
	sout_t * p_sout = (sout_t *)handle;
	
	sout_input_t * p_input = 0;
	unsigned char * p_extra = 0;
	
	if ( i_extra )
		p_extra = (unsigned char*)((jbyte*)(*env)->GetByteArrayElements(env,extra,0));
	
	if ( type == TYPE_H264_VIDEO )
	{
		es_format_Init(&p_sout->fmt_video_in,VIDEO_ES,VLC_CODEC_H264);
		p_sout->fmt_video_in.i_extra = i_extra;
		p_sout->fmt_video_in.p_extra = p_extra;
		p_input = soutAddStream( p_sout->p_mux, &p_sout->fmt_video_in);
		p_sout->p_video_input = p_input;	
		p_sout->i_video_pts_increment = (int64_t)((double)1000000.0 / frame_rate );	
		LOGI("Jxstreaming video stream\n");
	}
	else
	{
		unsigned char buf[2];

		int i_profile=2, i_sample_rate_idx = 4, i_channels=2;
		
		es_format_Init(&p_sout->fmt_audio_in,AUDIO_ES,VLC_CODEC_MP4A);
		
		p_sout->fmt_audio_in.i_extra = 2;
		p_sout->fmt_audio_in.p_extra = buf;
		
		 ((uint8_t *)p_sout->fmt_audio_in.p_extra)[0] =
            (i_profile + 1) << 3 | (i_sample_rate_idx >> 1);
        ((uint8_t *)p_sout->fmt_audio_in.p_extra)[1] =
            ((i_sample_rate_idx & 0x01) << 7) | (i_channels <<3);
		
		p_input = soutAddStream( p_sout->p_mux, &p_sout->fmt_audio_in);
		p_sout->p_audio_input = p_input;
		LOGI("Jxstreaming audio stream\n");
		
	}    
	if ( p_extra )
		(*env)->ReleaseByteArrayElements(env,extra,p_extra,0);
	return (jlong) p_input;	
}


jlong 
Java_com_smartvision_jxvideoh264_Jxstreaming_createVideoEncoder(JNIEnv * env,
																jobject this,
																jlong handle,
																jint type,
																jint rate,
																jint width,
																jint height,
																jint fps,
																jint gop)
{
	LOGI("Jxstreaming create video encoder start");

	sout_t * p_sout = (sout_t *)handle;
	video_encoder_t *p_enc;
	if ( !p_sout ) return 0;
	p_sout->p_video_enc = p_enc = malloc( sizeof( video_encoder_t) );
	if ( !p_enc ) return 0;
	p_enc->p_sout = p_sout; 
	p_enc->param = (x264_param_t*)malloc(sizeof(x264_param_t));
	p_enc->picture = (x264_picture_t*)malloc(sizeof(x264_picture_t));
	//x264_param_default(p_enc->param);
	x264_param_default_preset(p_enc->param, "veryfast" , "zerolatency" );
	x264_param_apply_profile(p_enc->param,"baseline");

	p_enc->param->i_log_level = X264_LOG_NONE;
	p_enc->param->i_width = width;
	p_enc->param->i_height = height;
		
	#if USE_YUV_NV12
	p_enc->param->i_csp = X264_CSP_NV21;
	#else
	p_enc->param->i_csp = X264_CSP_I420;
	#endif
	p_enc->param->i_fps_num = fps;
	p_enc->param->i_fps_den = 1;
	
	p_enc->param->rc.b_mb_tree = 0;
	p_enc->param->i_keyint_max = fps * 2 ;

	p_enc->param->i_bframe = 0;	
	
	p_enc->param->rc.i_bitrate = rate/1000;
	p_enc->param->rc.i_rc_method = X264_RC_CRF;
	p_enc->param->rc.i_vbv_max_bitrate=(int)((rate*1.2)/1000) ;
	p_enc->param->b_repeat_headers = 1;
	p_enc->f_fps = fps;
	p_enc->param->b_annexb = 1;

	if ((p_enc->handle = x264_encoder_open(p_enc->param))==0)
	{

	  LOGI("could not open codec !");
	  free(p_enc->param );
	  p_enc->param = 0;
	  free(p_enc->picture );
	  p_enc->picture = 0;
	  return 0;
	}
	#if USE_YUV_NV12 
	x264_picture_alloc( p_enc->picture,X264_CSP_NV21,width,height);
	#else
	x264_picture_alloc( p_enc->picture,X264_CSP_I420,width,height);
	#endif
	
	p_enc->p_pic_y = p_enc->picture->img.plane[0];
	p_enc->p_pic_uv = p_enc->picture->img.plane[1];
	
	p_enc->i_dts = VLC_TS_0;
	
	
	es_format_Init(&p_enc->fmt_out,VIDEO_ES,VLC_CODEC_H264);
	int i, i_nal,bytes = 0;
	x264_nal_t    *nal;
	p_enc->fmt_out.i_extra = 4 * width,height + 1000 ;
	p_enc->fmt_out.i_extra = x264_encoder_headers(p_enc->handle, &nal, &i_nal )  ;


	if ( p_enc->fmt_out.i_extra > 0 ) {
	
		p_enc->fmt_out.p_extra = malloc( p_enc->fmt_out.i_extra );
	
		for( i = 0; i < i_nal; i++ )
		{

			memcpy(p_enc->fmt_out.p_extra + bytes , nal[i].p_payload,nal[i].i_payload);
		    bytes+=nal[i].i_payload;
		    LOGI("xh264 encode header %d,encode nal %d",p_enc->fmt_out.i_extra
												   ,nal[i].i_payload);
	
		}

		p_enc->fmt_out.i_extra=bytes;										   
	}else{
		p_enc->fmt_out.p_extra = 0;
		p_enc->fmt_out.i_extra = 0;
		LOGI("xh264 encode header error");
	}

	p_enc->p_in = soutAddStream(p_sout->p_mux,&p_enc->fmt_out);

	LOGI("Jxstreaming create video encoder end");
	
	return (jlong)p_enc;  	 
}

static int h264_frame_type( int i_nal_type )
{
	int i_frame_type = i_nal_type;
    /* slice_type */
    switch( i_nal_type )
    {
    case 0: case 5:
        i_frame_type = BLOCK_FLAG_TYPE_P;
        break;
    case 1: case 6:
        i_frame_type = BLOCK_FLAG_TYPE_B;
        break;
    case 2: case 7:
        i_frame_type = BLOCK_FLAG_TYPE_I;
        break;
    case 3: case 8: /* SP */
        i_frame_type = BLOCK_FLAG_TYPE_P;
        break;
    case 4: case 9:
        i_frame_type = BLOCK_FLAG_TYPE_I;
        break;
    default:
        i_frame_type = 0;
        break;
    }
	return i_frame_type;
}

jint Java_com_smartvision_jxvideoh264_Jxstreaming_send( JNIEnv *env,
														jobject this,
														jlong h_sout,
														jlong h_input,
														jbyteArray p_frame,
														jint i_frame_length)
{
	sout_t *p_sys = ( sout_t *)h_sout;
	sout_input_t * p_input = ( sout_input_t *)h_input;
	unsigned char * p_es_data = (unsigned char*)((jbyte*)(*env)->GetByteArrayElements(env,p_frame,0));
	
	block_t *p_es = block_Alloc( i_frame_length );
	memcpy( p_es->p_buffer,p_es_data,i_frame_length );
	p_es->i_buffer = i_frame_length;
	p_es->i_length = 0;
	if ( p_sys->p_video_input == p_input )
	{
		p_es->i_pts = VLC_TS_0 + p_sys->i_video_pts;
		p_es->i_dts = VLC_TS_0 + p_sys->i_video_pts;
		p_sys->i_dts = p_sys->i_video_pts+=p_sys->i_video_pts_increment;	

		p_es->i_flags|=h264_frame_type(p_es->p_buffer[4] & 0x1f);
		
		//LOGI("Jxstreaming video mux :frame type %d , size %d",p_es->i_flags,p_es->i_buffer );
		
	}
	else if (p_sys->p_audio_input == p_input)	
	{
		if (p_sys->i_audio_pts_increment == 0){
			p_sys->i_audio_pts_increment = (int64_t)((double)90000.0 * 1024) / 44100 ;
		}
		
		p_es->i_pts = VLC_TS_0 + p_sys->i_audio_pts;
		p_es->i_dts = VLC_TS_0 + p_sys->i_audio_pts;	
		//p_es->i_length = p_sys->i_audio_pts_increment;
		p_sys->i_audio_pts += p_sys->i_audio_pts_increment;	
		//LOGI("Jxstreaming send :%d",i_frame_length);
	}
//	LOGI("Jxstreaming send :%d",i_frame_length);

	p_es->p_owner = p_input;
	pthread_mutex_lock(&p_sys->lock);
	block_FifoPut(p_sys->p_es_fifo,p_es);
	pthread_mutex_unlock(&p_sys->lock);
	//sout_block_mux( p_input, p_es );
	
	(*env)->ReleaseByteArrayElements(env,p_frame,p_es_data,0);	
	return i_frame_length;
}

jint Java_com_smartvision_jxvideoh264_Jxstreaming_encodeVideo(JNIEnv *env,
													jobject this,
													 jlong handle,
													 jbyteArray input )
{
  video_encoder_t *p_enc = ( video_encoder_t *)handle;
  int n_nal = -1;
  x264_picture_t pic_out;
  int i = 0,bytes = 0;
  int linesize = p_enc->param->i_width * p_enc->param->i_height;
  unsigned char * yuv = (unsigned char*)((jbyte*)(*env)->GetByteArrayElements(env,input,0));
#if USE_YUV_NV12
  unsigned char *p = yuv;
  p_enc->picture->img.plane[0] = p;
  p_enc->picture->img.plane[1] = p + linesize;
#else
  unsigned char * y = p_enc->picture->img.plane[0];
  unsigned char * u = p_enc->picture->img.plane[2];
  unsigned char * v = p_enc->picture->img.plane[1];
  memcpy(y,yuv,linesize);
  for ( i=0 ; i < linesize / 4  ; i++ )
	{
	  *(u+i) = *(yuv+linesize+i*2);
	  *(v+i) = *(yuv+linesize+i*2+1);
	}
#endif  
  p_enc->picture->i_type = X264_TYPE_AUTO;

  int i_result = x264_encoder_encode(p_enc->handle,&p_enc->p_nals,&n_nal,p_enc->picture,&pic_out); 
  if ( i_result < 0 ){
	  (*env)->ReleaseByteArrayElements(env,input,yuv,0);
	  return 0; 
  }

  block_t *p_es = block_Alloc(i_result);
  for( i = 0 ; i < n_nal ; i++ ){
	  memcpy(p_es->p_buffer + bytes, p_enc->p_nals[i].p_payload,p_enc->p_nals[i].i_payload);
	  bytes+=p_enc->p_nals[i].i_payload;
  }
  
  p_es->i_buffer = bytes;
  p_es->i_pts = VLC_TS_0 + p_enc->i_dts;
  p_es->i_dts = VLC_TS_0 + p_enc->i_dts;
  p_es->i_length = 0;//(int64_t)((double)1000000.0 / p_enc->f_fps );
  p_es->p_owner = p_enc->p_in;
  p_enc->p_sout->i_dts = p_enc->i_dts += (int64_t)((double)1000000.0 / p_enc->f_fps );

  int i_h264_slice_type = pic_out.i_type;
  if( i_h264_slice_type == H264_SLICE_TYPE_IDR || i_h264_slice_type == H264_SLICE_TYPE_I )
		p_es->i_flags |= BLOCK_FLAG_TYPE_I;
  else if( i_h264_slice_type == H264_SLICE_TYPE_P )
		p_es->i_flags |= BLOCK_FLAG_TYPE_P;
  else if( i_h264_slice_type == H264_SLICE_TYPE_B )
		p_es->i_flags |= BLOCK_FLAG_TYPE_B;

  if ( p_enc->p_in ){
	  pthread_mutex_lock(&p_enc->p_sout->lock);
	  block_FifoPut(p_enc->p_sout->p_es_fifo,p_es);
	  pthread_mutex_unlock(&p_enc->p_sout->lock);
	  //sout_block_mux(p_enc->p_in,p_es);
//	  LOGI("encode video %d\n",p_es->i_buffer);
  }else
	block_Release(p_es);
	
  (*env)->ReleaseByteArrayElements(env,input,yuv,0);
  return bytes;
}

static
void video_encoder_destroy( video_encoder_t * p_enc ){
	
	LOGI("Jxstreaming encoder destroy");
	if ( p_enc != NULL )
	{
	  if ( p_enc->picture ){
		p_enc->picture->img.plane[0] = p_enc->p_pic_y;
		p_enc->picture->img.plane[1] = p_enc->p_pic_uv;

		x264_picture_clean(p_enc->picture);
		p_enc->picture = 0;
	  }	  
	  LOGI("Jxstreaming encoder destroy clean picture");
	  if ( p_enc->param )
		free(p_enc->param);
	  p_enc->param = 0;

	  LOGI("Jxstreaming encoder destroy free param");
	  
	  if (p_enc->handle)
		x264_encoder_close(p_enc->handle);
	  p_enc->handle = 0;
	  
	  LOGI("Jxstreaming encoder destroy close handle");

	  if ( p_enc->fmt_out.p_extra )
		free(p_enc->fmt_out.p_extra);
	  p_enc->fmt_out.p_extra = 0;
	  if ( p_enc->p_in )
		soutDelStream(p_enc->p_sout->p_mux,p_enc->p_in);
		
	  LOGI("Jxstreaming encoder destroy delete stream");
	
	  p_enc->p_in = 0;
	  free(p_enc);
	  p_enc = 0;
	}		
	LOGI("Jxstreaming encoder destroy Ended");
	
}

jint Java_com_smartvision_jxvideoh264_Jxstreaming_destroy(JNIEnv *env,
													  jobject this,
													  jlong handle)
{
  sout_t * p_sout = (sout_t*)handle;
  if ( p_sout != NULL )
	{		
	  block_t * p_end = block_Alloc(1);
	  p_end->i_flags = BLOCK_FLAG_PREROLL;
	  block_FifoPut(p_sout->p_es_fifo,block_Duplicate(p_end));
	  block_FifoPut(p_sout->p_fifo,p_end);
	  pthread_mutex_lock(&p_sout->lock);
	  p_sout->b_closed = true;
	  pthread_mutex_unlock(&p_sout->lock);
	  pthread_join(p_sout->thread_mux,NULL);
	  pthread_join(p_sout->thread,NULL);
	  block_FifoRelease(p_sout->p_fifo);
	  block_FifoRelease(p_sout->p_es_fifo);
	  pthread_mutex_destroy(&p_sout->lock);
	  LOGI("Jxstreaming destroy send thread ended");
	  	  
	  if ( p_sout->p_video_input )
		soutDelStream(p_sout->p_mux,p_sout->p_video_input);
	  p_sout->p_video_input = 0;

	  if ( p_sout->p_audio_input )
		soutDelStream(p_sout->p_mux,p_sout->p_audio_input);
	  p_sout->p_audio_input = 0;
	  	
	  if ( p_sout->p_video_enc )
		video_encoder_destroy(p_sout->p_video_enc);
	  if ( p_sout->p_mux )
		soutClose(p_sout->p_mux);
	  free(p_sout);
	  p_sout = 0;
	}	
  return 0;
}

