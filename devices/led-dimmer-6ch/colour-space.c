/*

   Copyright (C) 2014 Stephen Robinson
  
   This file is part of Myha
  
   Myha is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 2 of the License, or
   (at your option) any later version.
  
   Myha is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this code (see the file names COPYING).  
   If not, see <http://www.gnu.org/licenses/>.
  
*/

#include <stdint.h>
#include "colour-space.h"

#define MIN(x, y)  ((x) < (y) ? (x) : (y))
#define MAX(x, y)  ((x) > (y) ? (x) : (y))

void rgb2hsv(uint8_t r, uint8_t g, uint8_t b, uint8_t* h, uint8_t* s, uint8_t* v)
{
  uint8_t min = MIN(r, MIN(g, b));
  uint8_t max = MAX(r, MAX(g, b));
  uint16_t diff = max - min;
  int16_t hue;

  *v = max;
  if(max != 0)
  {
    *s = diff * 255 / max;
    if(max == r)
      hue = (g - b) * 42 / diff;
    else if(max == g)
      hue = 85 + (b - r) * 42 / diff;
    else
      hue = 170 + (r - g) * 42 / diff;
    if(hue < 0)
      hue += 255;
    *h = hue;
  }
  else
  {
    *s = 0;
    *h = 0;
  }
}

void hsv2rgb(uint8_t h, uint8_t s, uint8_t v, uint8_t* r, uint8_t* g, uint8_t* b)
{
  uint16_t hh = h, ss = s, vv = v;
  uint8_t i = (hh * 6) >> 8;
  uint16_t f = (hh * 6) & 0xFF;
  uint16_t p = (vv * (0x100 - ss)) >> 8;
  uint16_t q = (vv * (0x100 - ((f * ss) >> 8))) >> 8;
  uint16_t t = (vv * (0x100 - (((0x100 - f) * ss) >> 8))) >> 8;

  switch(i)
  {
    case 0:
    case 6: *r = vv; *g = t; *b = p; break;
    case 1: *r = q; *g = vv; *b = p; break;
    case 2: *r = p; *g = vv; *b = t; break;
    case 3: *r = p; *g = q; *b = vv; break;
    case 4: *r = t; *g = p; *b = vv; break;
    case 5: *r = vv; *g = p; *b = q; break;
  }
}
