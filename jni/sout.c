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


#include "tsmux/tsmux.h"
#include "tsmux/vlc_block.h"

#include "x264/x264.h"

// Slice type define
#define H264_SLICE_TYPE_IDR           0x0001
#define H264_SLICE_TYPE_I             0x0002
#define H264_SLICE_TYPE_P             0x0003
#define H264_SLICE_TYPE_BREF          0x0004  /* Non-disposable B-frame */
#define H264_SLICE_TYPE_B             0x0005

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
	float f_fps;
	es_format_t fmt_out;
	sout_input_t * p_in;
	sout_t * p_sout;	
	
}video_encoder_t;

typedef struct sout_t
{
	int fd_udp;
	bool b_closed;
	pthread_mutex_t lock;
	pthread_t thread;
	video_encoder_t *p_video_enc;
	block_fifo_t * p_fifo;
	sout_mux_t * p_mux;	
};

static void h264_ts_callback(void * p_private, unsigned char * p_ts_data , size_t i_size )
{
	sout_t * p_sout = (sout_t *) p_private;
	
	block_t * p_ts = block_Alloc( i_size );
	memcpy( p_ts->p_buffer,p_ts_data, i_size);
	block_FifoPut(p_sout->p_fifo,p_ts);	
//	fprintf( stderr , " Receive h264 ts stream :%d\n",i_size);
}

void send_thread( void * arg)
{	
	sout_t * p_sout = (sout_t*)arg;
	bool closed = false;
	uint16_t i_pos = 0;
	unsigned char send_buffer[TS_SENDER_BUFFER_SIZE];
	pthread_mutex_lock(&p_sout->lock);
	closed = p_sout->b_closed;
	pthread_mutex_unlock(&p_sout->lock);
	
	LOGI("Jxstreaming send thread started");
	while(!closed)
	{				
		block_t *p_block_in = block_FifoGet(p_sout->p_fifo);		
		if ( p_block_in && p_block_in->i_flags & BLOCK_FLAG_PREROLL )
		{
			int i_buffer = p_block_in->i_buffer;
			unsigned char * pbuffer = p_block_in->p_buffer;
			while(i_buffer)
			{
				int i_copy = __MIN ( TS_SENDER_BUFFER_SIZE - i_pos , i_buffer );
				memcpy( (send_buffer + i_pos), pbuffer , i_copy );
				//printf("buffer %d , copy %d \n",i_buffer,i_copy);
				i_pos += i_copy;
				pbuffer += i_copy;
				i_buffer -= i_copy;
				if ( i_pos == TS_SENDER_BUFFER_SIZE )
				{
					int bytes = send(p_sout->fd_udp,send_buffer,i_pos,0); 
					i_pos = 0 ;
					LOGI("Jxstreaming send to %d",bytes );
				}
			}	
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
	sout_t* p_sys  = NULL;
	const char * addr = NULL;
	struct sockaddr_in addr_server;
	socklen_t addr_len = sizeof(struct sockaddr_in);
	p_sys = malloc( sizeof (sout_t));

	if ( p_sys == NULL )
		return 0;

	p_sys->p_mux = soutOpen(NULL,h264_ts_callback,p_sys);
	p_sys->p_video_enc = NULL;

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
		LOGI("error in connecting");
		close(p_sys->fd_udp);
	}	
	p_sys->b_closed = false;
	pthread_mutex_init(&p_sys->lock,NULL);
	
	pthread_create(&p_sys->thread,0,send_thread,(void*)p_sys);
	
	return (jlong)p_sys;
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
	sout_t * p_sout = (sout_t *)handle;
	video_encoder_t *p_enc;
	if ( !p_sout ) return 0;
	p_sout->p_video_enc = p_enc = malloc( sizeof( video_encoder_t) );
	if ( !p_enc ) return 0;
	p_enc->p_sout = p_sout; 
	p_enc->param = (x264_param_t*)malloc(sizeof(x264_param_t));
	p_enc->picture = (x264_picture_t*)malloc(sizeof(x264_picture_t));
	//x264_param_default(p_enc->param);
	x264_param_default_preset(p_enc->param, "fast" , "zerolatency" );
	x264_param_apply_profile(p_enc->param,"baseline");
	p_enc->param->i_log_level = X264_LOG_NONE;
	p_enc->param->i_width = width;
	p_enc->param->i_height = height;
	p_enc->param->rc.i_lookahead = 0;
	#if USE_YUV_NV12
	p_enc->param->i_csp = X264_CSP_NV21;
	#else
	p_enc->param->i_csp = X264_CSP_I420;
	#endif
	p_enc->param->i_fps_num = fps;
	p_enc->param->i_fps_den = 1;
	p_enc->param->i_keyint_max = gop;
	p_enc->param->rc.i_bitrate = rate/1000;
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
	
	p_enc->fmt_out.i_codec = VLC_CODEC_H264;
	p_enc->fmt_out.i_cat = VIDEO_ES; 
	
	int i, i_nal,bytes = 0;
	x264_nal_t    *nal;
	p_enc->fmt_out.i_extra = x264_encoder_headers(p_enc->handle, &nal, &i_nal );
	if ( p_enc->fmt_out.i_extra > 0 ) {
		p_enc->fmt_out.p_extra = malloc( p_enc->fmt_out.i_extra );
		for( i = 0; i < i_nal; i++ )
		{
			memcpy(p_enc->fmt_out.p_extra+bytes,nal[i].p_payload,nal[i].i_payload);
		    bytes+=nal[i].i_payload;
		}
	}else{
		p_enc->fmt_out.p_extra = 0;
		p_enc->fmt_out.i_extra = 0;
		LOGI("xh264 encode header error");
	}
	p_enc->p_in = soutAddStream(p_sout,&p_enc->fmt_out);
	
	return (jlong)p_enc;  	 
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
  p_enc->picture->img.plane[0]=p;
  p_enc->picture->img.plane[1]=p+linesize;
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
  p_es->i_pts = pic_out.i_pts;
  p_es->i_dts = pic_out.i_dts;
  p_es->i_length = (int64_t)((double)1000000.0 / p_enc->f_fps );
  int i_h264_slice_type = pic_out.i_type;
	if( i_h264_slice_type == H264_SLICE_TYPE_IDR || i_h264_slice_type == H264_SLICE_TYPE_I )
		p_es->i_flags |= BLOCK_FLAG_TYPE_I;
	else if( i_h264_slice_type == H264_SLICE_TYPE_P )
		p_es->i_flags |= BLOCK_FLAG_TYPE_P;
	else if( i_h264_slice_type == H264_SLICE_TYPE_B )
		p_es->i_flags |= BLOCK_FLAG_TYPE_B;

  if ( p_enc->p_in ){
	  sout_block_mux(p_enc->p_in,p_es);
  }else
	block_Release(p_es);
	
  (*env)->ReleaseByteArrayElements(env,input,yuv,0);
  return bytes;
}

static
void video_encoder_destroy( video_encoder_t * p_enc ){
	if ( p_enc != NULL )
	{
	  if ( p_enc->picture )
		x264_picture_clean(p_enc->picture);
	  p_enc->picture = 0;
	  if ( p_enc->param )
		free(p_enc->param);
	  p_enc->param = 0;
	  if (p_enc->handle)
		x264_encoder_close(p_enc->handle);
	  if ( p_enc->fmt_out.p_extra )
		free(p_enc->fmt_out.p_extra);
	  if ( p_enc->p_in )
		soutDelStream(p_enc->p_sout,p_enc->p_in);
	  free(p_enc);
	  p_enc = 0;
	}		
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
	  block_FifoPut(p_sout->p_fifo,p_end);	  
	  pthread_mutex_lock(&p_sout->lock);
	  p_sout->b_closed = true;
	  pthread_mutex_unlock(&p_sout->lock);
	  pthread_join(p_sout->thread,NULL);
	  block_FifoRelease(p_sout->p_fifo);
	  pthread_mutex_destroy(&p_sout->lock);
	  
	  if ( p_sout->p_video_enc )
		video_encoder_destroy(p_sout->p_video_enc);
	  
	  free(p_sout);
	  p_sout = 0;
	}	
  return 0;
}

