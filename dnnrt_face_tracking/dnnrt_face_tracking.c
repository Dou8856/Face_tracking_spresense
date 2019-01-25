/****************************************************************************
 * camera/camera_main.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <errno.h>
#include <debug.h>
#include <fcntl.h>
#include <time.h>
#include <semaphore.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/fs/mkfatfs.h>
#include "video/video.h"

#include <sys/ioctl.h>
#include <sys/boardctl.h>
#include <sys/mount.h>

#include <arch/chip/pm.h>
#include <arch/board/board.h>
#include <arch/chip/cisif.h>

#ifdef CONFIG_EXAMPLES_DNNRT_FACE_TRACKING_OUTPUT_LCD
#include <nuttx/lcd/lcd.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include "nximage.h"

#  ifdef CONFIG_IMAGEPROC
#    include <imageproc/imageproc.h>
#  endif
#endif

#include <dnnrt/runtime.h>
#include "loader_nnb.h"
#include "pnm_util.h"
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Display of vsync timing */
/* #define CAMERA_MAIN_CISIF_INTRTRACE */

/* Note: Buffer size must be multiple of 32. */

#define IMAGE_YUV_SIZE     (320*240*2) /* QVGA YUV422 */

//#define VIDEO_BUFNUM       (3)
#define VIDEO_BUFNUM       (1)

#define DEFAULT_REPEAT_NUM (10000000000000)

#ifdef CONFIG_EXAMPLES_DNNRT_FACE_TRACKING_OUTPUT_LCD
#ifndef CONFIG_EXAMPLES_CAMERA_LCD_DEVNO
#  define CONFIG_EXAMPLES_CAMERA_LCD_DEVNO 0
#endif

#define itou8(v) ((v) < 0 ? 0 : ((v) > 255 ? 255 : (v)))
#endif /* CONFIG_EXAMPLES_DNNRT_FACE_TRACKING_OUTPUT_LCD */

/****************************************************************************
 * Private Types
 ****************************************************************************/
struct uyvy_s
{
  uint8_t u0;
  uint8_t y0;
  uint8_t v0;
  uint8_t y1;
};

struct v_buffer {
  uint32_t             *start;	 
  uint32_t             length;
};
typedef struct v_buffer v_buffer_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct v_buffer  *buffers_video;
static unsigned int     n_buffers;

#ifdef CONFIG_EXAMPLES_DNNRT_FACE_TRACKING_OUTPUT_LCD
struct nximage_data_s g_nximage =
{
  NULL,          /* hnx */
  NULL,          /* hbkgd */
  0,             /* xres */
  0,             /* yres */
  false,         /* havpos */
  { 0 },         /* sem */
  0              /* exit code */
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
#ifdef CONFIG_EXAMPLES_DNNRT_FACE_TRACKING_OUTPUT_LCD
static inline int nximage_initialize(void)
{
  FAR NX_DRIVERTYPE *dev;
  nxgl_mxpixel_t color;
  int ret;

  /* Initialize the LCD device */

  printf("nximage_initialize: Initializing LCD\n");
  ret = board_lcd_initialize();
  if (ret < 0)
    {
      printf("nximage_initialize: board_lcd_initialize failed: %d\n", -ret);
      return ERROR;
    }

  /* Get the device instance */

  dev = board_lcd_getdev(CONFIG_EXAMPLES_CAMERA_LCD_DEVNO);
  if (!dev)
    {
      printf("nximage_initialize: board_lcd_getdev failed, devno=%d\n",
             CONFIG_EXAMPLES_CAMERA_LCD_DEVNO);
      return ERROR;
    }

  /* Turn the LCD on at 75% power */

  (void)dev->setpower(dev, ((3*CONFIG_LCD_MAXPOWER + 3)/4));

  /* Then open NX */

  printf("nximage_initialize: Open NX\n");
  g_nximage.hnx = nx_open(dev);
  if (!g_nximage.hnx)
    {
      printf("nximage_initialize: nx_open failed: %d\n", errno);
      return ERROR;
    }

  /* Set background color to black */

  color = 0;
  nx_setbgcolor(g_nximage.hnx, &color);
  ret = nx_requestbkgd(g_nximage.hnx, &g_nximagecb, NULL);
  if (ret < 0)
    {
      printf("nximage_initialize: nx_requestbkgd failed: %d\n", errno);
      nx_close(g_nximage.hnx);
      return ERROR;
    }

  while (!g_nximage.havepos)
    {
      (void) sem_wait(&g_nximage.sem);
    }
  printf("nximage_initialize: Screen resolution (%d,%d)\n",
         g_nximage.xres, g_nximage.yres);

  return 0;
}

#  ifndef CONFIG_IMAGEPROC
static inline void ycbcr2rgb(uint8_t y,  uint8_t cb, uint8_t cr,
                             uint8_t *r, uint8_t *g, uint8_t *b)
{
  int _r;
  int _g;
  int _b;
  _r = (128 * (y-16) +                  202 * (cr-128) + 64) / 128;
  _g = (128 * (y-16) -  24 * (cb-128) -  60 * (cr-128) + 64) / 128;
  _b = (128 * (y-16) + 238 * (cb-128)                  + 64) / 128;
  *r = (int)itou8(_r)*0.3;
  *g = (int)itou8(_g)*0.5;
  *b = (int)itou8(_b)*0.2;
}

static inline uint16_t ycbcrtorgb565(uint8_t y, uint8_t cb, uint8_t cr)
{
  uint8_t r;
  uint8_t g;
  uint8_t b;

  ycbcr2rgb(y, cb, cr, &r, &g, &b);
  r = (r >> 3) & 0x1f;
  g = (g >> 2) & 0x3f;
  b = (b >> 3) & 0x1f;
  return (uint16_t)(((uint16_t)r << 11) | ((uint16_t)g << 5) | (uint16_t)b);
}

/* Color conversion to show on display devices. */

static void yuv2rgb(void *buf, uint32_t size)
{
  struct uyvy_s *ptr;
  struct uyvy_s uyvy;
  uint16_t *dest;
  uint32_t i;

  ptr = buf;
  dest = buf;
  for (i = 0; i < size / 4; i++)
    {
      /* Save packed YCbCr elements due to it will be replaced with
       * converted color data.
       */

      uyvy = *ptr++;

      /* Convert color format to packed RGB565 */

      *dest++ = ycbcrtorgb565(uyvy.y0, uyvy.u0, uyvy.v0);
      *dest++ = ycbcrtorgb565(uyvy.y1, uyvy.u0, uyvy.v0);
    }
}
#  endif /* !CONFIG_IMAGEPROC */
#endif /* CONFIG_EXAMPLES_DNNRT_FACE_TRACKING_OUTPUT_LCD */

static int camera_prepare(int                fd,
                          enum v4l2_buf_type type,
                          uint32_t           buf_mode,
                          uint32_t           pixformat,
                          uint16_t           hsize,
                          uint16_t           vsize,
                          uint8_t            buffernum)
{
  int ret;
  int cnt;
  uint32_t fsize;
  struct v4l2_format         fmt = {0};
  struct v4l2_requestbuffers req = {0};
  struct v4l2_buffer         buf = {0};
  struct v_buffer  *buffers;

  /* VIDIOC_REQBUFS initiate user pointer I/O */

  req.type   = type;
  req.memory = V4L2_MEMORY_USERPTR;
  req.count  = buffernum;
  req.mode   = buf_mode;

	printf("buffernum = %d\n", req.count);
  
  ret = ioctl(fd, VIDIOC_REQBUFS, (unsigned long)&req);
  if (ret < 0)
    {
      printf("Failed to VIDIOC_REQBUFS: errno = %d\n", errno);
      return ret;
    }

  /* VIDIOC_S_FMT set format */

  fmt.type                = type;
  fmt.fmt.pix.width       = hsize;
  fmt.fmt.pix.height      = vsize;
  fmt.fmt.pix.field       = V4L2_FIELD_ANY;
  fmt.fmt.pix.pixelformat = pixformat;
	
  ret = ioctl(fd, VIDIOC_S_FMT, (unsigned long)&fmt);
  if (ret < 0)
    {
      printf("Failed to VIDIOC_S_FMT: errno = %d\n", errno);
      return ret;
    }

  /* VIDIOC_QBUF enqueue buffer */
      buffers_video = malloc(sizeof(v_buffer_t) * buffernum);
      buffers = buffers_video;

  if (!buffers)
    {
      printf("Out of memory\n");
      return ret;
    }

    fsize = IMAGE_YUV_SIZE;

  for (n_buffers = 0; n_buffers < buffernum; ++n_buffers)
    {
      buffers[n_buffers].length = fsize;

      /* Note: VIDIOC_QBUF set buffer pointer. */
      /*       Buffer pointer must be 32bytes aligned. */

      buffers[n_buffers].start  = memalign(32, fsize);
      if (!buffers[n_buffers].start)
        {
          printf("Out of memory\n");
          return ret;
        }
    }

  for (cnt = 0; cnt < n_buffers; cnt++)
    {
      memset(&buf, 0, sizeof(v4l2_buffer_t));
      buf.type = type;
      buf.memory = V4L2_MEMORY_USERPTR;
      buf.index = cnt;
      buf.m.userptr = (unsigned long)buffers[cnt].start;
      buf.length = buffers[cnt].length;

      ret = ioctl(fd, VIDIOC_QBUF, (unsigned long)&buf);
      if (ret)
        {
          printf("Fail QBUF %d\n", errno);
          return ret;;
        }
    }

  /* VIDIOC_STREAMON start stream */

  ret = ioctl(fd, VIDIOC_STREAMON, (unsigned long)&type);
  if (ret < 0)
    {
      printf("Failed to VIDIOC_STREAMON: errno = %d\n", errno);
      return ret;
    }

  return OK;
}

static void free_buffer(struct v_buffer  *buffers, uint8_t bufnum)
{
  uint8_t cnt;
  if (buffers)
    {
      for (cnt = 0; cnt < bufnum; cnt++)
        {
          if (buffers[cnt].start)
            {
              free(buffers[cnt].start);
            }
        }

      free(buffers);
    }
}

#define DNN_NNB_PATH "/mnt/sd0/dnnrt_model/face_recog.nnb"

/* Functions for dnnrt*/
typedef struct
  {
    char *nnb_path;
    bool skip_norm;
  } setting_t;

static void parse_args(int argc, char *argv[], setting_t *setting) {
	// Parse option
	int opt;
	while ((opt = getopt(argc, argv, "s"))!= -1) {
            switch (opt) {
            case 's': /*skip normalization*/
                setting->skip_norm = true;
                break;
            }
        }
   // setting-> nnb_path = (optind < argc) 
    setting -> nnb_path = DNN_NNB_PATH;
    if(setting->skip_norm) {
        printf("Image normalization skipped\n");
    }else{
        printf ("Image normalization enabled\n");
    }
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/
#ifdef CONFIG_BUILD_KERNEL
int dnnrt_face_tracking_main(int argc, FAR char *argv[])
#else
int dnnrt_face_tracking_main(int argc, char *argv[])
#endif
{
  int ret;
  int exitcode = ERROR;
  int v_fd;
  struct stat stat_buf;
  uint32_t loop;
  uint32_t buf_type;
  uint32_t format;
  struct v4l2_buffer         buf;
  uint8_t * gray_buf_ptr; 

  gray_buf_ptr = malloc(IMAGE_YUV_SIZE/2);
  // Declaration for DNNRT
  float *output_buffer, norm_factor;
  dnn_runtime_t rt;
  nn_network_t *network;
  setting_t setting = {0};
  parse_args (argc, argv, &setting);
  norm_factor = setting.skip_norm ? 1.0f : 255.0f;
  
  // Step 0: Load nnb file
  network = alloc_nnb_network(setting.nnb_path);
  printf("nnb path = %s\n", setting.nnb_path);
  
  if (network == NULL){
      printf("load nnb file failed\n");
      goto dnn_error;
  }
  else{
      printf("Netwoek is loaded\n");
  }
  
  //Step 1: initialize the dnn system

  ret = dnn_initialize(NULL);

  if(ret){
      printf("dnnrt_initialize faile; error code %d\n", ret);
      goto rt_error;
  }
  
  //Step 2: instantiate the loaded network as a dnn_runtime_bject
  /*dnn_runtime crashes*/
  //ret = dnn_runtime_initialize(&rt, network);

  if(ret){
      printf("network dnn runtime instantiate failed; error code: %d\n", ret);
      goto rt_error;
  }



  /* select capture mode */

#ifdef CONFIG_EXAMPLES_DNNRT_FACE_TRACKING_OUTPUT_LCD
  ret = nximage_initialize();
  if (ret < 0)
    {
      printf("camera_main: Failed to get NX handle: %d\n", errno);
      return ERROR;
    }
#  ifdef CONFIG_IMAGEPROC
  imageproc_initialize();
#  endif
#endif /* CONFIG_EXAMPLES_DNNRT_FACE_TRACKING_OUTPUT_LCD */

      buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      format   = V4L2_PIX_FMT_UYVY;

      loop = DEFAULT_REPEAT_NUM;

  ret = video_initialize("/dev/video");
  if (ret != 0)
    {
      printf("ERROR: Failed to initialize video: errno = %d\n", errno);
      goto errout_with_nx;
    }

  v_fd = open("/dev/video", 0);
  if (v_fd < 0)
    {
      printf("ERROR: Failed to open video.errno = %d\n", errno);
      goto errout_with_video_init;
    }

  /* Prepare VIDEO_CAPTURE */

  ret = camera_prepare(v_fd,
                       V4L2_BUF_TYPE_VIDEO_CAPTURE,
                       V4L2_BUF_MODE_RING,
                       V4L2_PIX_FMT_UYVY,
                       VIDEO_HSIZE_QVGA,
                       VIDEO_VSIZE_QVGA,
                       VIDEO_BUFNUM);
  if (ret < 0)
    {
      goto errout_with_buffer;
    }

  while (loop-- > 0)
    {
      /* Note: VIDIOC_DQBUF acquire capture data. */

      memset(&buf, 0, sizeof(v4l2_buffer_t));
      buf.type = buf_type;
      buf.memory = V4L2_MEMORY_USERPTR;

      ret = ioctl(v_fd, VIDIOC_DQBUF, (unsigned long)&buf);
      

      if (ret)
        {
          printf("Fail DQBUF %d\n", errno);
          goto errout_with_buffer;
        }


#ifdef CONFIG_EXAMPLES_DNNRT_FACE_TRACKING_OUTPUT_LCD
      if (format == V4L2_PIX_FMT_UYVY)
        {
          /* Convert YUV color format to RGB565 */
#  ifdef CONFIG_IMAGEPROC

//get 8bits image buffer for NNabla
	imageproc_convert_yuv2gray((void *)buf.m.userptr, 
		(void*)gray_buf_ptr, 
		VIDEO_HSIZE_QVGA, 
		VIDEO_VSIZE_QVGA);
//convert image buffer to rgb
	imageproc_convert_yuv2rgb(
		(void *)buf.m.userptr,
		VIDEO_HSIZE_QVGA, 
		VIDEO_VSIZE_QVGA);

    int i = 0;
    uint16_t* pixel_ptr = buf.m.userptr;
    // Convert to gray, not necessary
while (i < IMAGE_YUV_SIZE/2)
{	
    uint16_t r = ((*pixel_ptr >> 11) & (0X1F));
    uint16_t g = (*pixel_ptr >> 5) & (0X3F);
    uint16_t b = (*pixel_ptr) & (0X1F);
    uint16_t gray = g/2;
    //Overwrite the color buffer
    *pixel_ptr = (gray <<11 | gray << 5 | gray);
    i++;
    pixel_ptr ++;
}

#  else
          yuv2rgb((void *)buf.m.userptr, buf.bytesused);
#  endif
         nximage_image(g_nximage.hbkgd, (void *)buf.m.userptr);
        }
#endif /* CONFIG_EXAMPLES_DNNRT_FACE_TRACKING_OUTPUT_LCD */

      /* Note: VIDIOC_QBUF reset released buffer pointer. */

      ret = ioctl(v_fd, VIDIOC_QBUF, (unsigned long)&buf);
	
      if (ret)
        {
          printf("Fail QBUF %d\n", errno);
          goto errout_with_buffer;
        }
    }

  exitcode = OK;

errout_with_buffer:
  close(v_fd);

  free_buffer(buffers_video, VIDEO_BUFNUM);

errout_with_video_init:

  video_uninitialize();

errout_with_nx:
#ifdef CONFIG_EXAMPLES_DNNRT_FACE_TRACKING_OUTPUT_LCD
#  ifdef CONFIG_IMAGEPROC
  imageproc_finalize();
#  endif
  nx_close(g_nximage.hnx);
#endif /* CONFIG_EXAMPLES_DNNRT_FACE_TRACKING_OUTPUT_LCD */

fin: 
dnn_runtime_finalize(&rt);
rt_error:
dnn_finalize();
dnn_error:
destroy_nnb_network(network);
  return exitcode;
}
