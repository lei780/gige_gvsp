
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <string>
#include <iostream>

#include <unistd.h>
#include <errno.h>
#include <sched.h>
#include <signal.h>

#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <boost/random.hpp>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

//#include <SDL2/SDL.h>

#define BOUNDARY "boundarydonotcross"

#define STD_HEADER "Connection: close\r\n" \
    "Server: MJPG-Streamer/0.2\r\n" \
    "Cache-Control: no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0\r\n" \
    "Pragma: no-cache\r\n" \
    "Expires: Mon, 3 Jan 2000 12:34:56 GMT\r\n"


using namespace boost::interprocess;

uint32_t loop = 0;
uint32_t IMAGE_WIDTH = 1280;
uint32_t IMAGE_HEIGHT = 720;

#if 0
SDL_Window *sdlScreen;
SDL_Renderer *sdlRenderer;
SDL_Texture *sdlTexture;
SDL_Rect sdlRect;

SDL_DisplayMode display_mode;


void sdl2_init()
{
  int display_index;
  int err;

  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER))
  {
      std::cerr << "Could not initialize SDL - " << SDL_GetError() << std::endl;
      return ;
  }

  SDL_SetHint("SDL_HINT_RENDER_SCALE_QUALITY", "1");

  sdlScreen = SDL_CreateWindow("Simple YUV Window",
                SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                IMAGE_WIDTH, IMAGE_HEIGHT,
                SDL_WINDOW_SHOWN|SDL_WINDOW_RESIZABLE);
  if (!sdlScreen) {
      std::cerr << "SDL: could not create window - exiting: " << SDL_GetError() << std::endl;
      return ;
  }

  display_index = SDL_GetWindowDisplayIndex(sdlScreen);

  err = SDL_GetDesktopDisplayMode(display_index, &display_mode);
  if(!err)
  {
      printf("RENDER: video display %i ->  %dx%dpx @ %dhz\n",
          display_index,
          display_mode.w,
          display_mode.h,
          display_mode.refresh_rate);
  }
  else
      fprintf(stderr, "RENDER: Couldn't determine display mode for video display %i\n", display_index);

  printf("RENDER: setting window size to %ix%i\n", IMAGE_WIDTH, IMAGE_HEIGHT);
  SDL_SetWindowSize(sdlScreen, IMAGE_WIDTH, IMAGE_HEIGHT);

  sdlRenderer = SDL_CreateRenderer(sdlScreen, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (sdlRenderer == NULL) {
      fprintf(stderr, "SDL_CreateRenderer Error\n");
      return ;
  }

  if(SDL_RenderSetLogicalSize(sdlRenderer, IMAGE_WIDTH, IMAGE_HEIGHT) < 0){
      fprintf(stdout, "Could not set logical size of the SDL window renderer: %s\n", SDL_GetError());
  }

  SDL_RenderSetLogicalSize(sdlRenderer, IMAGE_WIDTH, IMAGE_HEIGHT);
  SDL_SetRenderDrawBlendMode(sdlRenderer, SDL_BLENDMODE_NONE);

  SDL_ShowWindow(sdlScreen);
  sdlTexture = SDL_CreateTexture(sdlRenderer, SDL_PIXELFORMAT_YUY2,
                        SDL_TEXTUREACCESS_STREAMING, IMAGE_WIDTH, IMAGE_HEIGHT);

  sdlRect.w = IMAGE_WIDTH;
  sdlRect.h = IMAGE_HEIGHT;
}

static void frame_rendering(void *pframe, int length)
{
    SDL_SetRenderDrawColor(sdlRenderer, 0, 0, 0, 255); /*black*/
    SDL_RenderClear(sdlRenderer);
 
    //printf("length : %d, image_width*2 : %d \n", length, IMAGE_WIDTH*2);
    //SDL_UpdateTexture(sdlTexture, &sdlRect, pframe, IMAGE_WIDTH*2);
    SDL_UpdateTexture(sdlTexture, &sdlRect, pframe, 1920*2);
 
    SDL_RenderCopy(sdlRenderer, sdlTexture, NULL, &sdlRect);
    SDL_RenderPresent(sdlRenderer);
}
#endif

void print_affinity() 
{
    cpu_set_t mask;
    long nproc, i;

    if (sched_getaffinity(0, sizeof(cpu_set_t), &mask) == -1) {
        perror("sched_getaffinity");
        assert(false);
    }
    nproc = sysconf(_SC_NPROCESSORS_ONLN);
    printf("sched_getaffinity = ");
    for (i = 0; i < nproc; i++) {
        printf("%d ", CPU_ISSET(i, &mask));
    }
    printf("\n");
}


void handle_signal(int _signal)
{

    if (_signal == SIGINT ) {
        printf("SIGINT \n");
        loop = 0;
    }
    else if( _signal == SIGTERM){
        printf("SIGTERM \n");
        loop = 0;
    }

}



int main(int argc, char *argv[])
{
    //SDL_Event e;
    uint32_t ids = 0;
    uint32_t prev_idx = 0;
    uint32_t frame_length = 0;

    shared_memory_object shm;
    char *shared_mem;

    struct timeval timestamp;

#if 0
    cpu_set_t mask;

    print_affinity() ;
    CPU_ZERO(&mask);
    CPU_SET(0, &mask);

    if (sched_setaffinity(0, sizeof(cpu_set_t), &mask) == -1) {
        perror("sched_setaffinity");
        //assert(false);
    }

    CPU_SET(1, &mask);
    if (sched_setaffinity(0, sizeof(cpu_set_t), &mask) == -1) {
        perror("sched_setaffinity");
        //assert(false);
    }

    print_affinity() ;
#endif

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    //Map the whole shared memory in this process
    try{
        //Open already created shared memory object.
        //shared_memory_object shm(open_only, "MySharedMemory", read_only);
        shm = shared_memory_object(open_only, "MySharedMemory", read_only);
    }
    catch(interprocess_exception &ex){
        std::cout << "shared memory Exception: ";
        std::cout << ex.what() << std::endl;

        return 1;
    }

    mapped_region region(shm, read_only);

    //sdl2_init();

    shared_mem = static_cast<char*>(region.get_address());
    loop = 1;

    //std::cout << "region address: " << std::hex << region.get_address() << std::endl;
    std::cout << "region size: " << std::dec << region.get_size() << std::endl;

    printf("HTTP/1.0 200 OK\r\n" \
           "Access-Control-Allow-Origin: *\r\n" \
           STD_HEADER \
           "Content-Type: multipart/x-mixed-replace;boundary=" BOUNDARY "\r\n" \
           "\r\n" \
           "--" BOUNDARY "\r\n");

    //printf("Content-Type: multipart/x-mixed-replace; boundary=--jpgboundary \r\n\r\n");

    while(loop == 1)
    {
        ids = *((uint32_t *)shared_mem);
        if( ids != prev_idx ) 
        {
#if 0 
            printf("frame index: 0x%d \n", ids);
            frame_length = *( (uint32_t *)(&shared_mem[sizeof(uint32_t)]) );
            printf("frame length: 0x%d \n", frame_length);

            timestamp = *( (struct timeval *)(&shared_mem[sizeof(uint32_t)*2]) );
            printf("frame timestamp: %ld.%06ld \n", timestamp.tv_sec, timestamp.tv_usec);
#else
            //printf("Content-type: image/jpeg\r\n");
            //frame_length = *((uint32_t *)(shared_mem+sizeof(uint32_t)));
            //printf("Content-length: %d\r\n\r\n", frame_length);
            //fwrite(shared_mem+8, 1, frame_length, stdout);
            //printf("\r\n\r\n\r\n");

            //frame_length = *((uint32_t *)(shared_mem+sizeof(uint32_t)));

            frame_length = *( (uint32_t *)(&shared_mem[sizeof(uint32_t)]) );
            timestamp = *( (struct timeval *)(&shared_mem[sizeof(uint32_t)*2]) );

            printf("Content-Type: image/jpeg\r\n" \
                   "Content-Length: %d\r\n" \
                   "X-Timestamp: %d.%06d\r\n" \
                   "\r\n", frame_length, (int)timestamp.tv_sec, (int)timestamp.tv_usec);

            fwrite(&shared_mem[(sizeof(uint32_t)*2)+sizeof(struct timeval)], 1, frame_length, stdout);

            printf("\r\n--" BOUNDARY "\r\n");
#endif

            //frame_rendering(shared_mem+8, region.get_size());
            prev_idx = ids;
            //break;
        }

#if 0
        if (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) { // click close icon then quit
                loop = 0;
            }

            if (e.type == SDL_KEYDOWN) {
                if (e.key.keysym.sym == SDLK_ESCAPE) // press ESC the quit
                loop = 0;
            }
        }
#endif
        //boost::this_thread::sleep(boost::posix_time::milliseconds(20));
    }

    return 0;
}
