/**
 * @file include/kobuki_driver/packet_handler/packet_finder.hpp
 *
 * @brief Simple packet finder
 *
 * Originally from the yujin control system suite (where it also has some
 * unit tests).
 *
 * Currently quite rough, could possibly be made general.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
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
 * @todo
 * (1) getting buffer from this class<br>
 * (2) simple construction of this class<br>
 * (3) device abstraction<br>
 * (4) extend-able packet structures<br>
 * (5) ros처럼 파일에서 스크립트읽고 auto-generation된 코드를 사용할 수 있으면 편하다.<br>
 * (6) packetFinder to ros converter<br>
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
