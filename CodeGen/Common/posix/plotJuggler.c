/*
  COPYRIGHT (C) 2009  Roberto Bucher (roberto.bucher@supsi.ch)

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
#include<stdio.h> 
#include<unistd.h>
#include<stdlib.h> 
#include<arpa/inet.h>
#include <netdb.h>
#include<sys/socket.h>
#include <string.h>

double get_run_time(void);

static void init(python_block *block)
{
  static struct sockaddr_in server;
  int s;

  char * IPbuf;
  struct hostent *he;
  const char *hostname = block->str;

#ifdef CG_WITH_ENV_HOST_ADDR
  if (hostname != NULL)
    {
      if (*hostname == '$')
        {
          const char *envname = hostname + 1;
          hostname = getenv(envname);
          if (hostname == NULL)
            {
              printf("environment variable \"%s\" not set\n", envname);
              exit(1);
            }
        }
    }
#endif /*CG_WITH_ENV_HOST_ADDR*/

  if ((he = gethostbyname(hostname)) == NULL)
     {
       printf("gethostbyname failure \"%s\" unresolved\n", hostname);
       exit(1);
     }
  IPbuf =  inet_ntoa(*((struct in_addr*) he->h_addr_list[0]));

  if ((s = socket(AF_INET, SOCK_DGRAM, 0)) < 0) exit(1);
  block->intPar[1] = s;

  server.sin_family      = AF_INET;
  server.sin_port         = htons(block->intPar[0]);
  server.sin_addr.s_addr = inet_addr(IPbuf);
  block->ptrPar = (void *) &server;
}

static void inout(python_block *block)
{
  int i;
  int * intPar    = block->intPar;
  double *u;
  double data[block->nin];
  char databuffer[15*block->nin+25];

  int s = intPar[1];
  double t = get_run_time();
  char strVal[12];
  
  for(i=0;i<block->nin;i++){
    u = block->u[i];
    data[i] = u[0];
  }

  sprintf(databuffer, "{\"ts\":%.3lf, \"y\":[", t);
  for(i=0;i<block->nin-1;i++){
    sprintf(strVal,"%.3lf, ", data[i]);
    strcat(databuffer, strVal);
  }
  sprintf(strVal,"%.3lf]}", data[i]);
  strcat(databuffer, strVal);
  /* printf("%s\n", databuffer); */
  
  sendto(s, databuffer, sizeof(databuffer) , 0 , (struct sockaddr *) block->ptrPar, sizeof(struct sockaddr_in));
}

static void end(python_block *block)
{
  int * intPar    = block->intPar;

  close(intPar[1]);
}

void plotJuggler(int flag, python_block *block)
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
