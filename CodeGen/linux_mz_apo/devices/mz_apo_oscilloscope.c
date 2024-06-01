/*
COPYRIGHT (C) 2024  David Storek (storedav@fel.cvut.cz)
 
Description: C-code for printing osciloscope-like behavior onto LCD screen.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
*/

#include <pyblock.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <pthread.h>

/* supporting files for MZ_APO board */
#include <shared_parlcd.h>
#include <mzapo_circular_queue.h>

/* parameters index definition */
#define REALPAR_IDX_X_MIN_VAL          0
#define REALPAR_IDX_X_MAX_VAL          1
#define REALPAR_IDX_Y_MIN_VAL          2
#define REALPAR_IDX_Y_MAX_VAL          3
#define REALPAR_IDX_STRENGTH_MIN_VAL   2
#define REALPAR_IDX_STRENGTH_MAX_VAL   3

#define INTPAR_IDX_LCD_X               0
#define INTPAR_IDX_LCD_Y               1
#define INTPAR_IDX_LCD_WIDTH           2
#define INTPAR_IDX_LCD_HEIGHT          3

#define INPUT_IDX_X_VAL                0
#define INPUT_IDX_Y_VAL                1
#define INPUT_IDX_STRENGTH_VAL         2

#define POINTS_BUFFER_SIZE             20

struct Params{
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double strength_min;
    double strength_max;

    int lcd_x;
    int lcd_y;
    int lcd_width;
    int lcd_height;
};

struct Input{
    double x;
    double y;
    double strength;
};

struct State{
    int lcd_user_id;
    struct Params pars;

    struct mzapo_cq points;
    pthread_mutex_t points_mutex;
};

struct Params read_params(python_block *block){
  struct Params pars;

  double * realPar = block->realPar;
  pars.x_min = realPar[REALPAR_IDX_X_MIN_VAL];
  pars.x_max = realPar[REALPAR_IDX_X_MAX_VAL];
  pars.y_min = realPar[REALPAR_IDX_Y_MIN_VAL];
  pars.y_max = realPar[REALPAR_IDX_Y_MAX_VAL];
  pars.strength_min = realPar[REALPAR_IDX_STRENGTH_MIN_VAL];
  pars.strength_max = realPar[REALPAR_IDX_STRENGTH_MAX_VAL];

  int * intPar = block->intPar;
  pars.lcd_x = intPar[INTPAR_IDX_LCD_X];
  pars.lcd_y = intPar[INTPAR_IDX_LCD_Y];
  pars.lcd_width = intPar[INTPAR_IDX_LCD_WIDTH];
  pars.lcd_height = intPar[INTPAR_IDX_LCD_HEIGHT];

  if(pars.lcd_width < 0 || pars.lcd_height < 0 ){
    fprintf(stderr,  "Display size must be positive!.\n");
    exit(1);
  }

  //  clip viewport to bounds of screen
  // first find slope of boundary values
  double x_slope = pars.lcd_width == 0 ? 0 : ((pars.x_max - pars.x_min) / pars.lcd_width);
  double x_q = pars.x_min - (x_slope * pars.lcd_x);
  double y_slope = pars.lcd_height == 0 ? 0 : ((pars.y_max - pars.y_min) / pars.lcd_height);
  double y_q = pars.y_min - (y_slope * pars.lcd_y);

  // clip left
  if(pars.lcd_x < 0){
    pars.x_min = x_slope * 0 + x_q;
    pars.lcd_width -= 0-pars.lcd_x;
    pars.lcd_width = pars.lcd_width < 0 ? 0 : pars.lcd_width;
    pars.lcd_x = 0;
  }
  // clip right
  if(pars.lcd_x + pars.lcd_width >= LCD_WIDTH){
    pars.x_max = x_slope * LCD_WIDTH + x_q;
    pars.lcd_width -= pars.lcd_x + pars.lcd_width - LCD_WIDTH;
    pars.lcd_width = pars.lcd_width < 0 ? 0 : pars.lcd_width;
  }
  // clip top
  if(pars.lcd_y < 0){
    pars.y_min = y_slope * 0 + y_q;
    pars.lcd_height -= 0-pars.lcd_y;
    pars.lcd_height = pars.lcd_height < 0 ? 0 : pars.lcd_height;
    pars.lcd_y = 0;
  }
  // clip bottom
  if(pars.lcd_y + pars.lcd_height >= LCD_HEIGHT){
    pars.y_max = y_slope * LCD_HEIGHT + y_q;
    pars.lcd_height -= pars.lcd_y + pars.lcd_height - LCD_HEIGHT;
    pars.lcd_height = pars.lcd_height < 0 ? 0 : pars.lcd_height;
  }  

  return pars;
}

struct Input read_and_clip_input(python_block *block, struct Params* pars){
  struct Input input;

  double *u = block->u[0];
  input.x = u[INPUT_IDX_X_VAL];
  input.y = u[INPUT_IDX_Y_VAL];
  input.strength = u[INPUT_IDX_STRENGTH_VAL];

  input.x = fmin(fmax(input.x, pars->x_min), pars->x_max);
  input.y = fmin(fmax(input.y, pars->y_min), pars->y_max);
  input.strength = fmin(fmax(input.strength, pars->strength_min), pars->strength_max);

  return input;
}

struct Point{
  uint16_t x; 
  uint16_t y;
  float val;
};

// function used to draw the oscilloscope curve. Runs in the LCD's thread.
void draw_function(void* _ctx){
  struct State* ctx = (struct State*) _ctx;
  struct Params pars = ctx->pars;

  // printf("Pars x: %d, y: %d, w: %d, h: %d\n", pars.lcd_x, pars.lcd_y, pars.lcd_width, pars.lcd_height);

  // draw frame around region
  uint16_t frame[LCD_WIDTH];
  memset(&frame[0], 0xffff, sizeof(uint16_t) * LCD_WIDTH);
  draw_funct_lcd_set_pixels(&frame[0], pars.lcd_x, pars.lcd_y, pars.lcd_width, 1);
  draw_funct_lcd_set_pixels(&frame[0], pars.lcd_x, pars.lcd_y + pars.lcd_height-1, pars.lcd_width, 1);
  draw_funct_lcd_set_pixels(&frame[0], pars.lcd_x, pars.lcd_y, 1, pars.lcd_height);
  draw_funct_lcd_set_pixels(&frame[0], pars.lcd_x + pars.lcd_width-1, pars.lcd_y, 1, pars.lcd_height );
}

/*
=================== Block functions ===================
*/

/*  INITIALIZATION FUNCTION  */
static void init(python_block *block)
{
  struct State * state = (struct State*)malloc(sizeof(struct State));
    /* Save memory map structure to ptrPar */
  block->ptrPar = state;

  // registerblock as user of LCD
  state->lcd_user_id = lcd_register_user(&draw_function, state);

  // Read parameters
  state->pars = read_params(block);

  // setup points queue and its access mutex
  cq_init(&state->points, POINTS_BUFFER_SIZE, sizeof(struct Point));
  int ret = pthread_mutex_init( &state->points_mutex, NULL);
  if(ret != 0){
    printf("Can't initialize mutex for accessing oscilloscope's points buffer. Errcode: %d\n", ret);
    return(-1);
  }
}

/*  INPUT/OUTPUT  FUNCTION  */
static void inout(python_block *block)
{
  struct State * state = block->ptrPar;
  state->pars = read_params(block);   
  struct Input input = read_and_clip_input(block, &state->pars);
}

/*  TERMINATION FUNCTION  */
static void end(python_block *block)
{
  struct State *state = (struct State*)block->ptrPar;

  // unregister block as user of LCD
  void* _ctx_to_delete = lcd_unregister_user(state->lcd_user_id); // forget returned context without deleting it, it is the state that is deleted later.

  // deinit points queue and its mutex
  pthread_mutex_destroy(&state->points_mutex);
  cq_deinit(&state->points);  

  // Free allocated memory
  free(state);
  state = NULL;
  block->ptrPar = NULL;
}

void mz_apo_oscilloscope(int flag, python_block *block)
{
  if (flag==CG_OUT){          /* input / output */
    inout(block);
  }
  else if (flag==CG_END){     /* termination */ 
    end(block);
  }
  else if (flag ==CG_INIT){    /* initialisation */
    init(block);
  }
}


