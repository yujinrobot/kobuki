/*
 * Copyright (c) 2012, Yujin Robot.
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
 *     * Neither the name of Yujin Robot nor the names of its
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
/**
 * @file /include/kobuki_driver/packet_finder.hhpp
 *
 * Originally from the yujin control system suite (where it also has some
 * unit tests).
 *
 * Currently quite rough, could possibly be made general.
 *
 * @date Jan, 2011
 *
 * @author jakan2
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef ROBOT_DATA_HPP_
#define ROBOT_DATA_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ecl/containers.hpp>
#include <stdint.h>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace packet_handler
{

/*****************************************************************************
 ** Interface
 *****************************************************************************/
/**
 * @brief
 * Provides simple packet finder which may be consist of stx, etx, payload, ...
 *
 */
class payloadBase
{
public:

  /**
   * this is simple magic to write the flag, when we get the packet from the host or
   * when we want to send the data
   */
  bool yes;

  /*
   * construct and destruct
   */
  payloadBase() : yes(false) {};
  virtual ~payloadBase() {};

  /*
   * serialisation
   */
  virtual bool serialise(ecl::PushAndPop<unsigned char> & byteStream)=0;
  virtual bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)=0;

  // utilities
  // todo; let's put more useful converters here. Or we may use generic converters
protected:
  // below funciton should be replaced wiht converter
  template<typename T>
    void buildVariable(T & V, ecl::PushAndPop<unsigned char> & buffer)
    {
      if (buffer.size() < sizeof(T))
        return;
      V = static_cast<unsigned char>(buffer.pop_front());

      unsigned int size_value(sizeof(T));
      for (unsigned int i = 1; i < size_value; i++)
      {
        V |= ((static_cast<unsigned char>(buffer.pop_front())) << (8 * i));
      }
    }

  template<typename T>
    void buildBytes(T & V, ecl::PushAndPop<unsigned char> & buffer)
    {
      unsigned int size_value(sizeof(T));
      for (unsigned int i = 0; i < size_value; i++)
      {
        buffer.push_back(static_cast<unsigned char>((V >> (i * 8)) & 0xff));
      }
    }
};

/**
 * Need to be very careful with this - it will only work across platforms if they
 * happen to be doing reinterpret_cast with the same float standard.
 * @param V
 * @param buffer
 */
template<>
inline   void payloadBase::buildVariable<float>(float & V, ecl::PushAndPop<unsigned char> & buffer)
  {
    if (buffer.size() < 4)
      return;
    unsigned int ui;
    ui = static_cast<unsigned char>(buffer.pop_front());

    unsigned int size_value(4);
    for (unsigned int i = 1; i < size_value; i++)
    {
      ui |= ((static_cast<unsigned char>(buffer.pop_front())) << (8 * i));
    }

    V = reinterpret_cast<float&>(ui);
  }

template<>
inline void payloadBase::buildBytes<float>(float & V, ecl::PushAndPop<unsigned char> & buffer)
  {
    if (buffer.size() < 4)
      return;
    unsigned int size_value(4);
    unsigned int ui(reinterpret_cast<unsigned int&>(V));
    for (unsigned int i = 0; i < size_value; i++)
    {
      buffer.push_back(static_cast<unsigned char>((ui >> (i * 8)) & 0xff));
    }
  }

//#define FRAC_MAX 9223372036854775807LL /* 2**63 - 1 */
//
//struct dbl_packed
//{
//    int exp;
//    long long frac;
//};
//
//void pack(double x, struct dbl_packed *r)
//{
//    double xf = fabs(frexp(x, &r->exp)) - 0.5;
//
//    if (xf < 0.0)
//    {
//        r->frac = 0;
//        return;
//    }
//
//    r->frac = 1 + (long long)(xf * 2.0 * (FRAC_MAX - 1));
//
//    if (x < 0.0)
//        r->frac = -r->frac;
//}
//
//double unpack(const struct dbl_packed *p)
//{
//    double xf, x;
//
//    if (p->frac == 0)
//        return 0.0;
//
//    xf = ((double)(llabs(p->frac) - 1) / (FRAC_MAX - 1)) / 2.0;
//
//    x = ldexp(xf + 0.5, p->exp);
//
//    if (p->frac < 0)
//        x = -x;
//
//    return x;
//}

}
;
// namespace packet_handler

#endif /* ROBOT_DATA_HPP_ */
