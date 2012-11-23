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
 * @file /include/kobuki_node/packet_finder.hhpp
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

#ifndef PACKET_FINDER_HPP_
#define PACKET_FINDER_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <iomanip>
#include <ecl/containers.hpp>
#include <ecl/sigslots.hpp>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki
{

/*****************************************************************************
 ** Using
 *****************************************************************************/

/*****************************************************************************
 ** Interface
 *****************************************************************************/
/**
 * @brief
 * Provides simple packet finder which may be consist of stx, etx, payload, ...
 *
 * @note
 *
 * <b>Usage</b>:
 *
 *
 * @code
 *
 *
 * @code
 *
 *
 * @endcode
 *
 *
 * @todo;
 * (1) getting buffer from this class
 * (2) simple construction of this class
 * (3) device abstraction
 * (4) extend-able packet structures
 * (5) ros처럼 파일에서 스크립트읽고 auto-generation된 코드를 사용할 수 있으면 편하다.
 * (6) packetFinder to ros converter
 * (7) evaluate it (valgrind)
 *
 */
class PacketFinderBase
{
public:
  typedef ecl::PushAndPop<unsigned char> BufferType;

  enum packetFinderState
  {
    clearBuffer = 0,
    waitingForStx,
    waitingForPayloadSize,
    waitingForPayloadToEtx,
    waitingForEtx,
  };
  enum packetFinderState state;
protected:

  unsigned int size_stx;
  unsigned int size_etx;
  unsigned int size_length_field;
  bool variable_size_payload;
  unsigned int size_max_payload;
  unsigned int size_payload;
  unsigned int size_checksum_field;

  BufferType STX;
  BufferType ETX;
  BufferType buffer;

  bool verbose;

  ecl::Signal<const std::string&> sig_warn, sig_error;

public:
  PacketFinderBase(); /**< Default constructor. Use with configure(). **/

  virtual ~PacketFinderBase() {};

  void configure(const std::string &sigslots_namespace,
                 const BufferType & putStx, const BufferType & putEtx, unsigned int sizeLengthField,
                 unsigned int sizeMaxPayload, unsigned int sizeChecksumField, bool variableSizePayload);
  void clear();
  void enableVerbose();
  virtual bool update(const unsigned char * incoming, unsigned int numberOfIncoming);
  virtual bool checkSum();
  unsigned int numberOfDataToRead();
  void getBuffer(BufferType & bufferRef);

protected:
  bool WaitForStx(const unsigned char datum);
  bool waitForPayloadSize(const unsigned char * incoming, unsigned int numberOfIncoming);
  bool waitForEtx(const unsigned char incoming, bool & foundPacket);
  bool waitForPayloadAndEtx(const unsigned char * incoming, unsigned int numberOfIncoming, bool & foundPacket);
};

}
;
// namespace kobuki

#endif /* PACKET_FINDER_HPP_ */
