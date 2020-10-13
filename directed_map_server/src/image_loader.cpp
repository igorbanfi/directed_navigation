/*
 * Copyright (c) 2020, Igor Banfi
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <cstring>
#include <stdexcept>

#include <stdlib.h>
#include <stdio.h>

// We use SDL_image to load the image from disk
#include <SDL/SDL_image.h>

// Use Bullet's Quaternion object to create one from Euler angles
#include <LinearMath/btQuaternion.h>

#include "directed_map_server/image_loader.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace directed_map_server
{

void
loadMapFromFile(directed_msgs::GetDirectedMap::Response* resp, const char* fnamexu, const char* fnamexd,
                const char* fnameyu, const char* fnameyd, double res, bool negate,
                double occ_th, double free_th, double* origin,
                MapMode mode)
{
  SDL_Surface* imgxu;
  SDL_Surface* imgxd;
  SDL_Surface* imgyu;
  SDL_Surface* imgyd;

  unsigned char* pixels;
  unsigned char* data_;
  unsigned char* p;
  unsigned char value;
  int rowstride, n_channels, avg_channels;
  unsigned int i,j, direction;
  int k;
  double occ;
  int alpha;
  int color_sum;
  double color_avg;

  // Load the image using SDL.  If we get NULL back, the image load failed.

  if(!(imgxu = IMG_Load(fnamexu)))
  {
    std::string errmsg = std::string("failed to open image file \"") +
            std::string(fnamexu) + std::string("\": ") + IMG_GetError();
    throw std::runtime_error(errmsg);
  }

  if(!(imgxd = IMG_Load(fnamexd)))
  {
    std::string errmsg = std::string("failed to open image file \"") +
            std::string(fnamexd) + std::string("\": ") + IMG_GetError();
    throw std::runtime_error(errmsg);
  }

  if(!(imgyu = IMG_Load(fnameyu)))
  {
    std::string errmsg = std::string("failed to open image file \"") +
            std::string(fnameyu) + std::string("\": ") + IMG_GetError();
    throw std::runtime_error(errmsg);
  }

  if(!(imgyd = IMG_Load(fnameyd)))
  {
    std::string errmsg = std::string("failed to open image file \"") +
            std::string(fnameyd) + std::string("\": ") + IMG_GetError();
    throw std::runtime_error(errmsg);
  }

  // Copy the image data into the map structure
  resp->map.info.width = imgxu->w;
  resp->map.info.height = imgxu->h;
  resp->map.info.resolution = res;
  resp->map.info.origin.position.x = *(origin);
  resp->map.info.origin.position.y = *(origin+1);
  resp->map.info.origin.position.z = 0.0;
  btQuaternion q;
  q.setEulerZYX(*(origin+2), 0, 0);
  resp->map.info.origin.orientation.x = q.x();
  resp->map.info.origin.orientation.y = q.y();
  resp->map.info.origin.orientation.z = q.z();
  resp->map.info.origin.orientation.w = q.w();

  // Allocate space to hold the data
  resp->map.dataXu.resize(resp->map.info.width * resp->map.info.height);
  resp->map.dataXd.resize(resp->map.info.width * resp->map.info.height);
  resp->map.dataYu.resize(resp->map.info.width * resp->map.info.height);
  resp->map.dataYd.resize(resp->map.info.width * resp->map.info.height);

  data_ = (unsigned char*) malloc (resp->map.info.width * resp->map.info.height+1);

  // Get values that we'll need to iterate through the pixels
  rowstride = imgxu->pitch;
  n_channels = imgxu->format->BytesPerPixel;

  // NOTE: Trinary mode still overrides here to preserve existing behavior.
  // Alpha will be averaged in with color channels when using trinary mode.
  if (mode==TRINARY || !imgxu->format->Amask)
    avg_channels = n_channels;
  else
    avg_channels = n_channels - 1;

  for (direction = 1; direction < 5; direction++)
  {
  // Copy pixel data into the map structure
  if (direction==1)
    pixels = (unsigned char*)(imgxu->pixels);
  else if (direction==2)
    pixels = (unsigned char*)(imgxd->pixels);
  else if (direction==3)
    pixels = (unsigned char*)(imgyu->pixels);
  else if (direction==4)
    pixels = (unsigned char*)(imgyd->pixels);
  for(j = 0; j < resp->map.info.height; j++)
  {
    for (i = 0; i < resp->map.info.width; i++)
    {
      // Compute mean of RGB for this pixel
      p = pixels + j*rowstride + i*n_channels;
      color_sum = 0;
      for(k=0;k<avg_channels;k++)
        color_sum += *(p + (k));
      color_avg = color_sum / (double)avg_channels;

      if (n_channels == 1)
          alpha = 1;
      else
          alpha = *(p+n_channels-1);

      if(negate)
        color_avg = 255 - color_avg;

      if(mode==RAW){
          value = color_avg;
          data_[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = value;
          continue;
      }


      // If negate is true, we consider blacker pixels free, and whiter
      // pixels occupied.  Otherwise, it's vice versa.
      occ = (255 - color_avg) / 255.0;

      // Apply thresholds to RGB means to determine occupancy values for
      // map.  Note that we invert the graphics-ordering of the pixels to
      // produce a map with cell (0,0) in the lower-left corner.
      if(occ > occ_th)
        value = +100;
      else if(occ < free_th)
        value = 0;
      else if(mode==TRINARY || alpha < 1.0)
        value = -1;
      else
        {
        double ratio = (occ - free_th) / (occ_th - free_th);
        value = 99 * ratio;
        }
        data_[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = value;
      }
    }

    //if (direction==0)
    //  memcpy(&resp->map.data[0], data_, resp->map.info.width * resp->map.info.height);
    if (direction==1)
      memcpy(&resp->map.dataXu[0], data_, resp->map.info.width * resp->map.info.height);
    else if (direction==2)
      memcpy(&resp->map.dataXd[0], data_, resp->map.info.width * resp->map.info.height);
    else if (direction==3)
      memcpy(&resp->map.dataYu[0], data_, resp->map.info.width * resp->map.info.height);
    else if (direction==4)
      memcpy(&resp->map.dataYd[0], data_, resp->map.info.width * resp->map.info.height);

  }

  SDL_FreeSurface(imgxu);
  SDL_FreeSurface(imgxd);
  SDL_FreeSurface(imgyu);
  SDL_FreeSurface(imgyd);
}

}
