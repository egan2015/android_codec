#include <jni.h>
#include <android/log.h>
#include <stdint.h>
#include "x264/x264.h"

#define  LOG_TAG    "libjxcodec"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define MAX_OUTPUT_BUFFER 100000

#define USE_YUV_NV12 1

typedef struct Jxcodec
{
  x264_t * handle;
  x264_param_t * param;
  x264_picture_t *picture;
  x264_nal_t *nal;
  uint8_t *picture_buf;
}Jxcodec;

jlong 
Java_com_smartvision_jxvideoh264_Jxaudio_create( JNIEnv *env,
												jobject this,
												jint type)
{
  Jxcodec * encoder = NULL;
  encoder = malloc( sizeof (Jxcodec));

  if ( encoder == NULL )
	return 0;
  return (jlong)encoder;
}

jint 
Java_com_smartvision_jxvideoh264_Jxaudio_decode(JNIEnv *env,
												jobject this,
												jlong handle,
												jbyteArray input,
												jint input_buffer_size,
												jbyteArray output,
												jint output_buffer_size)
{
  Jxcodec *codec = (Jxcodec*)handle;
  int out_size = output_buffer_size,bytes = 0;
  return out_size;
}

jint 
Java_com_smartvision_jxvideoh264_Jxaudio_destroy(JNIEnv *env,
												 jobject this,
												 jlong handle)
{
  Jxcodec * codec = (Jxcodec*)handle;
  if ( codec != NULL )
	{
	  free(codec);
	}	
  return 0;
}



jlong 
Java_com_smartvision_jxvideoh264_Jxcodec_create(JNIEnv * env,
												jobject this,
												jint type,
												jint rate,
												jint width,
												jint height,
												jint fps,
												jint gop)
{


  Jxcodec * encoder = NULL;
  encoder = malloc( sizeof (Jxcodec));

  if ( encoder == NULL )
	return 0;
  encoder->param = (x264_param_t*)malloc(sizeof(x264_param_t));
  encoder->picture = (x264_picture_t*)malloc(sizeof(x264_picture_t));
  //x264_param_default(encoder->param);
  x264_param_default_preset(encoder->param, "fast" , "zerolatency" );
  x264_param_apply_profile(encoder->param,"baseline");
  encoder->param->i_log_level = X264_LOG_NONE;
  encoder->param->i_width = width;
  encoder->param->i_height = height;
  encoder->param->rc.i_lookahead = 0;
#if USE_YUV_NV12
  encoder->param->i_csp = X264_CSP_NV21;
#else
  encoder->param->i_csp = X264_CSP_I420;
#endif
  encoder->param->i_fps_num = fps;
  encoder->param->i_fps_den = 1;
  encoder->param->i_keyint_max = gop;
  encoder->param->rc.i_bitrate = rate/1000;
  encoder->param->b_repeat_headers = 1;
  encoder->param->b_annexb = 1;

  if ((encoder->handle = x264_encoder_open(encoder->param))==0)
	{

	  LOGI("could not open codec !");
	  free(encoder->param );
	  encoder->param = 0;
	  free(encoder->picture );
	  encoder->picture = 0;
	  return 0;
	}
#if USE_YUV_NV12 
  x264_picture_alloc( encoder->picture,X264_CSP_NV21,width,height);
#else
  x264_picture_alloc( encoder->picture,X264_CSP_I420,width,height);
#endif
  return (jlong)encoder;
}
jint Java_com_smartvision_jxvideoh264_Jxcodec_encode(JNIEnv *env,
													 jobject this,
													 jlong handle,
													 jbyteArray input,
													 jbyteArray output,
													 jint output_buffer_size)
{
  Jxcodec * codec = (Jxcodec*)handle;

  int n_nal = -1;
  x264_picture_t pic_out;
  int i = 0,bytes = 0;
  int linesize = codec->param->i_width * codec->param->i_height;
  unsigned char * out_buf = (unsigned char*)((jbyte*)(*env)->GetByteArrayElements(env,output,0));
  unsigned char * yuv = (unsigned char*)((jbyte*)(*env)->GetByteArrayElements(env,input,0));
  unsigned char * buff = out_buf;
#if USE_YUV_NV12
  unsigned char *p = yuv;
  codec->picture->img.plane[0]=p;
  codec->picture->img.plane[1]=p+linesize;
#else
  unsigned char * y = codec->picture->img.plane[0];
  unsigned char * u = codec->picture->img.plane[2];
  unsigned char * v = codec->picture->img.plane[1];
  memcpy(y,yuv,linesize);
  for ( i=0 ; i < linesize / 4  ; i++ )
	{
	  *(u+i) = *(yuv+linesize+i*2);
	  *(v+i) = *(yuv+linesize+i*2+1);
	}
#endif  
  codec->picture->i_type = X264_TYPE_AUTO;
  if ( x264_encoder_encode(codec->handle,&codec->nal,&n_nal,codec->picture,&pic_out)<0){
	  (*env)->ReleaseByteArrayElements(env,input,yuv,0);
	  (*env)->ReleaseByteArrayElements(env,output,out_buf,0);
	  return 0; 
  }
  for( i = 0 ; i < n_nal ; i++ ){
	  LOGI("x264 encode nal %d,payload size %d",n_nal,codec->nal[i].i_payload);	
	  memcpy(buff,codec->nal[i].p_payload,codec->nal[i].i_payload);
	  buff+=codec->nal[i].i_payload;
	  bytes+=codec->nal[i].i_payload;
  }
  (*env)->ReleaseByteArrayElements(env,input,yuv,0);
  (*env)->ReleaseByteArrayElements(env,output,out_buf,0);
  return bytes;
}

jint Java_com_smartvision_jxvideoh264_Jxcodec_destroy(JNIEnv *env,
													  jobject this,
													  jlong handle)
{
  Jxcodec * codec = (Jxcodec*)handle;
  if ( codec != NULL )
	{
	  if ( codec->picture )
		x264_picture_clean(codec->picture);
	  codec->picture = 0;
	  if ( codec->param )
		free(codec->param);
	  codec->param = 0;
	  if (codec->handle)
		x264_encoder_close(codec->handle);
	  free(codec);
	  codec = 0;
	}	
  return 0;
}

