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

struct
{
  struct
  {
    char name[32];
    char location[32];
    uint8_t fade_rate;        // time to fade to new level in 10ths of a second
    uint8_t manual_fade_rate_min;
    uint8_t manual_fade_rate_max;
    uint8_t manual_fade_rate_inc;
    uint8_t scene_level[16];  // scene level in percent (0->100)
    uint8_t profile[64];

  } channel[NUM_DIMMER_CHANNELS];

} eeprom EEMEM = 
{
  .channel = 
  {
    {
      .name = "6 Channel LED Dimmer: Channel 1",
      .location = "",
      .fade_rate = 5,
      .scene_level = 
      {
        100, 100, 100, 100, 100, 100, 100, 100,
        100, 100, 100, 100, 100, 100, 100, 100
      },
      .profile = 
      {
          0,   1,   2,   3,   4,   5,   6,   7,
          9,  10,  11,  13,  14,  16,  18,  19,
         21,  23,  25,  27,  29,  31,  33,  35,
         38,  40,  43,  46,  48,  51,  54,  57,
         60,  64,  67,  71,  75,  79,  83,  87,
         91,  96, 100, 105, 110, 116, 121, 127,
        133, 139, 145, 152, 159, 166, 173, 181,
        189, 197, 206, 214, 224, 233, 243, 254
      }
    },
    {
      .name = "6 Channel LED Dimmer: Channel 2",
      .location = "",
      .fade_rate = 5,
      .profile = 
      {
          0,   1,   2,   3,   4,   5,   6,   7,
          9,  10,  11,  13,  14,  16,  18,  19,
         21,  23,  25,  27,  29,  31,  33,  35,
         38,  40,  43,  46,  48,  51,  54,  57,
         60,  64,  67,  71,  75,  79,  83,  87,
         91,  96, 100, 105, 110, 116, 121, 127,
        133, 139, 145, 152, 159, 166, 173, 181,
        189, 197, 206, 214, 224, 233, 243, 254
      }
    },
    {
      .name = "6 Channel LED Dimmer: Channel 3",
      .location = "",
      .fade_rate = 5,
      .profile = 
      {
          0,   1,   2,   3,   4,   5,   6,   7,
          9,  10,  11,  13,  14,  16,  18,  19,
         21,  23,  25,  27,  29,  31,  33,  35,
         38,  40,  43,  46,  48,  51,  54,  57,
         60,  64,  67,  71,  75,  79,  83,  87,
         91,  96, 100, 105, 110, 116, 121, 127,
        133, 139, 145, 152, 159, 166, 173, 181,
        189, 197, 206, 214, 224, 233, 243, 254
      }
    },
    {
      .name = "6 Channel LED Dimmer: Channel 4",
      .location = "",
      .fade_rate = 5,
      .scene_level = 
      {
        100, 100, 100, 100, 100, 100, 100, 100,
        100, 100, 100, 100, 100, 100, 100, 100
      },
      .profile = 
      {
          0,   1,   2,   3,   4,   5,   6,   7,
          9,  10,  11,  13,  14,  16,  18,  19,
         21,  23,  25,  27,  29,  31,  33,  35,
         38,  40,  43,  46,  48,  51,  54,  57,
         60,  64,  67,  71,  75,  79,  83,  87,
         91,  96, 100, 105, 110, 116, 121, 127,
        133, 139, 145, 152, 159, 166, 173, 181,
        189, 197, 206, 214, 224, 233, 243, 254
      }
    },
    {
      .name = "6 Channel LED Dimmer: Channel 5",
      .location = "",
      .fade_rate = 5,
      .profile = 
      {
          0,   1,   2,   3,   4,   5,   6,   7,
          9,  10,  11,  13,  14,  16,  18,  19,
         21,  23,  25,  27,  29,  31,  33,  35,
         38,  40,  43,  46,  48,  51,  54,  57,
         60,  64,  67,  71,  75,  79,  83,  87,
         91,  96, 100, 105, 110, 116, 121, 127,
        133, 139, 145, 152, 159, 166, 173, 181,
        189, 197, 206, 214, 224, 233, 243, 254
      }
    },
    {
      .name = "6 Channel LED Dimmer: Channel 6",
      .location = "",
      .fade_rate = 5,
      .profile = 
      {
          0,   1,   2,   3,   4,   5,   6,   7,
          9,  10,  11,  13,  14,  16,  18,  19,
         21,  23,  25,  27,  29,  31,  33,  35,
         38,  40,  43,  46,  48,  51,  54,  57,
         60,  64,  67,  71,  75,  79,  83,  87,
         91,  96, 100, 105, 110, 116, 121, 127,
        133, 139, 145, 152, 159, 166, 173, 181,
        189, 197, 206, 214, 224, 233, 243, 254
      }
    }
  }
};

