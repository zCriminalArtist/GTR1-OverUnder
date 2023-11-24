#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

#include "gifdec.h"

// not exposed in C++ yet.
extern "C" {
  int32_t               vexTaskAddWithArg( int (* callback)(void *), int interval, void *arg, char const *label );
}

namespace vex {
  class Gif {
    private:
      gd_GIF *_gif;
      int    _sx;
      int    _sy;
      void   *_gifmem;
      void   *_buffer;

    static int render(void *arg ) {
      if( arg == NULL)
        return(0);
        
      Gif *instance = static_cast<Gif *>(arg);

      gd_GIF *gif = instance->_gif;
    
      for (unsigned looped = 1;; looped++) {
          while (gd_get_frame(gif)) {
              int32_t now = vexSystemTimeGet();

              gd_render_frame(gif, (uint8_t *)instance->_buffer);
            
              int32_t ex = instance->_sx + gif->width - 1;
              int32_t ey = instance->_sy + gif->height - 1;
              ex  = (ex  > 479 ? 479 : ex  );
              ey  = (ey  > 239 ? 239 : ey  );
            
              vexDisplayCopyRect( instance->_sx, instance->_sy, ex, ey, (uint32_t *)instance->_buffer, gif->width);
            
              int32_t delay = gif->gce.delay * 10;
            
              int32_t delta = vexSystemTimeGet() - now;
              delay -= delta;
              if( delay > 0 )
                this_thread::sleep_for(delay);
          }
          if (looped == gif->loop_count)
              break;

          gd_rewind(gif);
      }
    
      free(instance->_buffer);
      gd_close_gif(gif);
      free(instance->_gifmem);
    
      return(0);
    }

    
    public:
      Gif( const char *fname, int sx, int sy ) {
        _sx = sx;
        _sy = sy;
        FILE *fp = fopen( fname, "rb" );

        if( fp != NULL ) {
          fseek( fp, 0, SEEK_END );
          size_t len = ftell( fp );
          fseek( fp, 0, SEEK_SET );

          _gifmem = malloc( len );
          
          if( _gifmem != NULL ) {
            int nRead = fread( _gifmem, 1, len, fp );
            (void) nRead;
          }
          fclose(fp);

          if( _gifmem != NULL ) {
            // create a FILE from memory buffer
            FILE *fp = fmemopen( _gifmem, len, "rb" );

            // open gof file
            // will allocate memory for background and one animation
            // frame.
            _gif = gd_open_gif( fp );
            if( _gif == NULL ) {
              return;
            }
            
            // memory for rendering frame
            _buffer = (uint32_t *)malloc(_gif->width * _gif->height * sizeof(uint32_t));
            if( _buffer == NULL ) {
              // out of memory
              gd_close_gif( _gif );
              free(_gifmem);
            }
            else {
              // create thread to handle this gif
              vexTaskAddWithArg( render, 2, static_cast<void *>(this), "GIF" );
            }
          }
        }
        
      };
    
      ~Gif() {};
    
  };
}