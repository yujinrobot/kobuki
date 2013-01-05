/**
 * @file /kobuki_driver/src/driver/packet_finder.cpp
 *
 * @brief Packet handling implementation.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/kobuki_driver/packet_handler/packet_finder.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Implementation
*****************************************************************************/

PacketFinderBase::PacketFinderBase() :
    state(waitingForStx), verbose(false)
{
}


/*****************************************************************************
** Public
*****************************************************************************/

void PacketFinderBase::configure(const std::string &sigslots_namespace,
               const BufferType & putStx, const BufferType & putEtx, unsigned int sizeLengthField,
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

  sig_warn.connect(sigslots_namespace + std::string("/ros_warn"));
  sig_error.connect(sigslots_namespace + std::string("/ros_error"));

  //todo; exception
  // Problem1: size_length_field = 1, vairable_size_payload = false

  clear();
}

void PacketFinderBase::clear()
{
  state = waitingForStx;
  buffer.clear();
}

void PacketFinderBase::enableVerbose()
{
  verbose = true;
}

bool PacketFinderBase::checkSum()
{
  return true;
}

unsigned int PacketFinderBase::numberOfDataToRead()
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

void PacketFinderBase::getBuffer(BufferType & bufferRef)
{
  bufferRef = buffer;
}

/**
 * Checks for incoming packets.
 *
 * @param incoming
 * @param numberOfIncoming
 * @return bool : true if a valid incoming packet has been found.
 */
bool PacketFinderBase::update(const unsigned char * incoming, unsigned int numberOfIncoming)
{
  // clearBuffer = 0, waitingForStx, waitingForPayloadSize, waitingForPayloadToEtx, waitingForEtx,
  // std::cout << "update [" << numberOfIncoming << "][" << state << "]" << std::endl;
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
          state = waitingForPayloadSize; // kobukibot
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
  if ( found_packet ) {
    return checkSum();	//what happen if checksum is equal to false(== -1)?
  } else {
    return false;
  }
}
/*****************************************************************************
** Protected
*****************************************************************************/

bool PacketFinderBase::WaitForStx(const unsigned char datum)
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

bool PacketFinderBase::waitForPayloadSize(const unsigned char * incoming, unsigned int numberOfIncoming)
{
  // push data
  unsigned char first_byte;
  for (unsigned int i = 0; i < numberOfIncoming; i++) {
    first_byte = incoming[i];
    buffer.push_back(incoming[i]);
  }

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
      case 1: // kobuki
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
      printf("[payloadSize: %d]\n", size_payload);
    }

    return true;
  }
}

bool PacketFinderBase::waitForEtx(const unsigned char incoming, bool & foundPacket)
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

bool PacketFinderBase::waitForPayloadAndEtx(const unsigned char * incoming, unsigned int numberOfIncoming, bool & foundPacket)
{
  // push data
  for (unsigned int i = 0; i < numberOfIncoming; i++)
  {
    buffer.push_back(incoming[i]);
  }
  /*********************
  ** Error Handling
  **********************/
  if ( size_payload > size_max_payload ) {
    state = clearBuffer;
    std::ostringstream ostream;
    ostream << "abnormally sized payload retrieved, clearing [" << size_max_payload << "][" << size_payload << "]";

    ostream << std::setfill('0') << std::uppercase; //declare once, use everytime
//	ostream.fill('0') //call once, affects everytime
//      ostream.width(2); //need to call everytime

/*
    ostream << "[";
    for (unsigned int i = 0; i < numberOfIncoming; ++i ) {
      ostream.width(2); // need to declare evertime
      ostream << std::hex << static_cast<int>(*(incoming+i)) << " " << std::dec;
    }
    ostream << "\b]";
*/
    ostream << ", buffer: [" << std::setw(2) << buffer.size() << "][";
    for (unsigned int i = 0; i < buffer.size(); ++i ) {
      ostream << std::setw(2) << std::hex << static_cast<int>(buffer[i]) << " " << std::dec;
    }
    ostream << "\b]";

    sig_error.emit(ostream.str());
    return false;
  }
  // check when we need to wait for etx
  if (buffer.size() < size_stx + size_length_field + size_payload + size_checksum_field + size_etx)
  {
    return false;
  }
  else
  {
    if (verbose) {
      std::cout << "Start check etx " << std::endl;
      for (unsigned int i = 0; i < numberOfIncoming; ++i ) {
        std::cout << std::hex << static_cast<int>(*(incoming+i)) << " ";
      }
      std::cout << std::dec << std::endl;
    }
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


} // namespace kobuki
