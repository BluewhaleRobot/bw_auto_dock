/******************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Bluewhale Robot
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Author: Xie fusheng, Randoms
 *******************************************************************************/

#ifndef _GVALUES_H_
#define _GVALUES_H_

#define MAXDOUBLE 1e1000
#ifdef LINUX
#include <values.h>
#endif
#ifdef MACOSX
#include <limits.h>
#include <math.h>
//#define isnan(x) (x==FP_NAN)
#endif
#ifdef _WIN32
#include <limits>
#ifndef __DRAND48_DEFINED__
#define __DRAND48_DEFINED__
inline double drand48()
{
    return double(rand()) / RAND_MAX;
}
#endif
#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif
#define round(d) (floor((d) + 0.5))
typedef unsigned int uint;
#define isnan(x) (_isnan(x))
#endif

#endif
