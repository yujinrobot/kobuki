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

#include <ecl/containers.hpp>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace packet_handler
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
class packetFinder
{
public:
  typedef ecl::PushAndPop<unsigned char> BufferType;

protected:
  enum packetFinderState
  {
    clearBuffer = 0,
    waitingForStx,
    waitingForPayloadSize,
    waitingForPayloadToEtx,
    waitingForEtx,
  };

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
  enum packetFinderState state;

  bool verbose;

public:
  packetFinder() :
      state(waitingForStx), verbose(false)
  {
  }

  packetFinder(const unsigned char putOneByteStx, const unsigned char putOneByteEtx, unsigned int sizeLengthField,
               unsigned int sizeMaxPayload, unsigned int sizeChecksumField, bool variableSizePayload) :
      size_stx(1), size_etx(1), size_length_field(sizeLengthField), variable_size_payload(variableSizePayload), size_max_payload(
          sizeMaxPayload), size_payload(variable_size_payload ? 0 : sizeMaxPayload), size_checksum_field(
          sizeChecksumField), STX(size_stx, putOneByteStx), ETX(size_etx, putOneByteEtx), buffer(
          size_stx, size_stx + size_length_field + size_max_payload + size_checksum_field + size_etx), state(
          waitingForStx), verbose(false)
  {
    clear();
  }

  packetFinder(const BufferType & putStx, const BufferType & putEtx, unsigned int sizeLengthField,
               unsigned int sizeMaxPayload, unsigned int sizeChecksumField, bool variableSizePayload) :
      size_stx(putStx.size()), size_etx(putEtx.size()), size_length_field(sizeLengthField), variable_size_payload(
          variableSizePayload), size_max_payload(sizeMaxPayload), size_payload(
          variable_size_payload ? 0 : sizeMaxPayload), size_checksum_field(sizeChecksumField), STX(putStx), ETX(putEtx), buffer(
          size_stx, size_stx + size_length_field + size_max_payload + size_checksum_field + size_etx), state(
          waitingForStx), verbose(false)
  {
    clear();
  }

  virtual ~packetFinder() {};

  void configure(const BufferType & putStx, const BufferType & putEtx, unsigned int sizeLengthField,
                 unsigned int sizeMaxPayload, unsigned int sizeChecksumField, bool variableSizePayload)
  {
    size_stx = putStx.size();
    size_etx = putEtx.size();
    size_length_field = sizeLengthField;
    variable_size_payload = variableSizePayload;
    size_max_payload = sizeMaxPayload;
    size_payload = variable_size_payload ? 0 : sizeMaxPayload;
    size_checksum_field = sizeChecksumField;
    STX = putStx;
    ETX = putEtx;
    buffer = BufferType(size_stx + size_length_field + size_max_payload + size_checksum_field + size_etx);
    state = waitingForStx;

    // std::cout << "install checksum operator " << std::endl;

    //todo; exception
    // Problem1: size_length_field = 1, vairable_size_payload = false

    clear();
  }

  void clear()
  {
    state = waitingForStx;
    buffer.clear();
  }

  void enableVerbose()
  {
    verbose = true;
  }

  virtual bool update(const unsigned char * incoming, unsigned int numberOfIncoming)
  {
    bool result = updatePacket(incoming, numberOfIncoming);
    return result;
  }

  virtual bool checkSum()
  {
    return true;
  }

  unsigned int numberOfDataToRead()
  {
    unsigned int num(0);

    switch (state)
    {
      case waitingForEtx:
        num = 1;
        break;

      case waitingForPayloadToEtx:
        num = size_payload + size_etx + size_checksum_field;
        break;

      case waitingForPayloadSize:
        num = size_checksum_field;
        break;

      case waitingForStx:
      case clearBuffer:
      default:
        num = 1;
        break;
    }

    if (verbose)
    {
      printf("[state(%d):%02d]", state, num);
    }
    return num;
  }

  void getBuffer(BufferType & bufferRef)
  {
    bufferRef = buffer;
  }

protected:
  bool updatePacket(const unsigned char * incoming, unsigned int numberOfIncoming)
  {
    if (!(numberOfIncoming > 0))
      return false;

    bool found_packet(false);

    if ( state == clearBuffer ) {
      buffer.clear();
      state = waitingForStx;
    }
    switch (state)
    {
      case waitingForStx:
        if (WaitForStx(incoming[0]))
        {
          if (size_length_field)
          {
            state = waitingForPayloadSize;
          }
          else
          {
            if (variable_size_payload)
            {
              // e.g. stargazer
              state = waitingForEtx;
            }
            else
            {
              // e.g. iroboQ
              //Todo; should put correct state
              state = waitingForPayloadToEtx;
            }
          }
        }
        break;
      case waitingForEtx:
        if (waitForEtx(incoming[0], found_packet))
        {
          state = clearBuffer;
        }
        break;

      case waitingForPayloadSize:
        if (waitForPayloadSize(incoming, numberOfIncoming))
        {
          state = waitingForPayloadToEtx;
        }

        break;

      case waitingForPayloadToEtx:
        if (waitForPayloadAndEtx(incoming, numberOfIncoming, found_packet))
        {
          state = clearBuffer;
        }
        break;

      default:
        state = waitingForStx;
        break;
    }

    return found_packet ? checkSum() : false;
  }

  bool WaitForStx(const unsigned char datum)
  {
    bool found_stx(true);

    // add incoming datum
    buffer.push_back(datum);

    // check whether we have STX
    for (unsigned int i = 0; i < buffer.size() && i < STX.size(); i++)
    {
      if (buffer[i] != STX[i])
      {
        found_stx = false;
        buffer.pop_front();
        break;
      }
    }

    return (found_stx && buffer.size() == STX.size());
  }

  bool waitForPayloadSize(const unsigned char * incoming, unsigned int numberOfIncoming)
  {
    // push data
    for (unsigned int i = 0; i < numberOfIncoming; i++)
      buffer.push_back(incoming[i]);

    if (verbose)
    {
      for (unsigned int i = 0; i < buffer.size(); i++)
        printf("%02x ", buffer[i]);
      printf("\n");
    }

    // check when we need to wait for etx
    if (buffer.size() < size_stx + size_length_field)
    {
      return false;
    }
    else
    {
      switch (size_length_field)
      {
        case 1:
          size_payload = buffer[size_stx];
          break;
        case 2:
          size_payload = buffer[size_stx];
          size_payload |= buffer[size_stx + 1] << 8;
          break;
        case 4:
          size_payload = buffer[size_stx];
          size_payload |= buffer[size_stx + 1] << 8;
          size_payload |= buffer[size_stx + 2] << 16;
          size_payload |= buffer[size_stx + 3] << 24;
          break;
        default:
          // put assertion failure
          size_payload = 1;
          break;
      }

      if (verbose)
      {
        printf("[payloadSize: %d]", size_payload);
      }

      return true;
    }
  }

  bool waitForEtx(const unsigned char incoming, bool & foundPacket)
  {
    // push data
    buffer.push_back(incoming);

    // check when we need to wait for etx
    // if minimum payload size is 1
    if (buffer.size() < size_stx + size_etx + 1)
    {
      return false;
    }
    else
    {
      unsigned int number_of_match(0);
      for (unsigned int i = 0; i < ETX.size(); i++)
      {
        if (buffer[buffer.size() - ETX.size() + i] == ETX[i])
        {
          number_of_match++;
        }
      }

      if (number_of_match == ETX.size())
      {
        foundPacket = true;
        return true;
      }

      if (buffer.size() >= size_stx + size_max_payload + size_etx)
        return true;
      else
        return false;
    }
  }

  bool waitForPayloadAndEtx(const unsigned char * incoming, unsigned int numberOfIncoming, bool & foundPacket)
  {
    // push data
    for (unsigned int i = 0; i < numberOfIncoming; i++)
    {
      buffer.push_back(incoming[i]);
    }

    // check when we need to wait for etx
    if (buffer.size() < size_stx + size_length_field + size_payload + size_checksum_field + size_etx)
    {
      return false;
    }
    else
    {
      if (verbose)
        std::cout << "Start check etx " << std::endl;
      foundPacket = true;

      for (unsigned int i = (size_stx + size_length_field + size_payload + size_checksum_field);
          i < (size_stx + size_length_field + size_payload + size_checksum_field + size_etx); i++)
      {
        if (buffer[i] != ETX[i])
        {
          foundPacket = false;
        }
      }
      if (verbose)
        std::cout << "End of checking etx " << std::endl;
      return true;
    }
  }
};

}
;
// namespace packet_handler

#endif /* PACKET_FINDER_HPP_ */
