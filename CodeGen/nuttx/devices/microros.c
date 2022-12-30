/*
COPYRIGHT (C) 2022  Roberto Bucher (roberto.bucher@supsi.ch)

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

extern double microros_input[4];
extern double microros_output[4];

static void init(python_block *block)
{
}

static void inout(python_block *block)
{
  int i;
  double * u;
  double * y;
  
  for(i=0; i<block->nin;i++){
    u = (double *) block->u[i];
    microros_output[i] = u[0];
  }
  for(i=0; i<block->nout;i++){
    y = (double *) block->y[i];
    y[0] = microros_input[i];
  }
}

static void end(python_block *block)
{
}

void microros(int flag, python_block *block)
{
  if (flag==CG_OUT){          /* get input */
    inout(block);
  }
  else if (flag==CG_END){     /* termination */ 
    end(block);
  }
  else if (flag ==CG_INIT){    /* initialisation */
    init(block);
  }
}


