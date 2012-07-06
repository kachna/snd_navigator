/*
 *      gap_nd_nav.cpp
 *
 *      Copyright 2009 Luca Invernizzi <invernizzi.l@gmail.com>
 *
 *      This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 2 of the License, or
 *      (at your option) any later version.
 *
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 *
 *      You should have received a copy of the GNU General Public License
 *      along with this program; if not, write to the Free Software
 *      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#ifndef GAP_ND_NAV_H_
#define GAP_ND_NAV_H_

#include "snd_data.h"

/// Normalize angle to domain -pi, pi
inline double normalize(double z)
{
  return atan2(sin(z), cos(z));
}

void main_algorithm(SND_data * robot);
//extern void *__dso_handle __attribute__ ((__visibility__ ("hidden")));

#endif //GAP_ND_NAV_H_
