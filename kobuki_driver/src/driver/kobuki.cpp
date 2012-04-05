/**
 * @file /cruizcore/src/driver/cruizcore.cpp
 *
 * @brief Implementation for the cruizcore device driver.
 *
 * @date 20/08/2010
 **/

/*****************************************************************************
<<<<<<< HEAD
 ** Includes
 *****************************************************************************/

#include <ecl/converters.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/geometry/angle.hpp>
#include <ecl/time/timestamp.hpp>
#include "../../include/kobuki_driver/kobuki.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki
{

/*****************************************************************************
 ** Using
 *****************************************************************************/
using ecl::Converter;
using ecl::TimeStamp;

/*****************************************************************************
** Implementation [PacketFinder]
*****************************************************************************/

bool PacketFinder::checkSum()
{
  unsigned int packet_size(buffer.size());
  unsigned char cs(0);
  for (unsigned int i = 2; i < packet_size; i++)
  {
    cs ^= buffer[i];
  }

  return cs ? false : true;
}


/*****************************************************************************
 ** Implementation [Kobuki]
 *****************************************************************************/

void Kobuki::init(Parameters &parameters) throw (ecl::StandardException)
{

  pubtime("init");

  if (!parameters.validate())
  {
    throw ecl::StandardException(LOC, ecl::ConfigurationError, "Kobuki's parameter settings did not validate.");
  }
  //gyro_data.header.frame_id = "mobile_base_gyro";
  protocol_version = parameters.protocol_version;

  //std::cout << "1 " << std::flush;
  serial.open(parameters.device_port, ecl::BaudRate_115200, ecl::DataBits_8, ecl::StopBits_1, ecl::NoParity);
  //serial.block(4000);
  serial.clear();
  //std::cout << "2 " << std::flush;

//	is_connected=true;

  std::string sigslots_namespace = parameters.sigslots_namespace;
  sig_wheel_state.connect(sigslots_namespace + std::string("/joint_state"));
  sig_sensor_data.connect(sigslots_namespace + std::string("/sensor_data"));
  //sig_serial_timeout.connect(sigslots_namespace+std::string("/serial_timeout"));

  sig_default.connect(sigslots_namespace + std::string("/default"));
  sig_ir.connect(sigslots_namespace + std::string("/ir"));
  sig_dock_ir.connect(sigslots_namespace + std::string("/dock_ir"));
  sig_inertia.connect(sigslots_namespace + std::string("/inertia"));
  sig_cliff.connect(sigslots_namespace + std::string("/cliff"));
  sig_current.connect(sigslots_namespace + std::string("/current"));
  sig_magnet.connect(sigslots_namespace + std::string("/magnet"));
  sig_hw.connect(sigslots_namespace + std::string("/hw"));
  sig_fw.connect(sigslots_namespace + std::string("/fw"));
  sig_time.connect(sigslots_namespace + std::string("/time"));
  sig_st_gyro.connect(sigslots_namespace + std::string("/st_gyro"));
  sig_eeprom.connect(sigslots_namespace + std::string("/eeprom"));
  sig_gp_input.connect(sigslots_namespace + std::string("/gp_input"));
  //sig_reserved0;

  is_running = true;
  is_connected = true;

  ecl::PushAndPop<unsigned char> stx(2, 0);
  ecl::PushAndPop<unsigned char> etx(1);
  stx.push_back(0xaa);
  stx.push_back(0x55);
  packet_finder.configure(stx, etx, 1, 64, 1, true);

  //std::cout << "is enable?" << std::endl;
  /******************************************
   ** Configuration & Connection Test
   *******************************************/

  last_tick_left = 0;
  last_tick_right = 0;
  last_rad_left = 0.0;
  last_rad_right = 0.0;
  last_mm_left = 0.0;
  last_mm_right = 0.0;

  v = 0;
  w = 0;
  radius = 0;
  speed = 0;
  bias = 0.298; //wheelbase, wheel_to_wheel, in [m]

  start();
}

void Kobuki::close()
{
  stop();
  ROS_WARN_STREAM("Device: Kobuki ROS Node: Terminated.");
  return;
}

/**
 * @brief Performs a scan looking for incoming data packets.
 *
 * Use this as the worker function in a loop in a thread (from outside this library).
 * You can retrieve timeout, invalid packet and raw data information from signals
 * associated with this class (see the class definition).
 *
 * Refer to CruizCoreNodelet for an example use case.
 *
 * @sa CruizCoreNodelet
 *
 * @return bool : success or failure of the scan.
 */

void Kobuki::runnable()
{
  unsigned char buf[256];
  bool get_packet;
  bool dummy_mode = false;

  while (is_running)
  {
    pubtime("every_tick");
    get_packet = false;

    /*********************
    ** Read Incoming
    **********************/
    int n(serial.read(buf, packet_finder.numberOfDataToRead()));

    ROS_DEBUG_STREAM("kobuki_node : serial_read(" << n << ")");
    if (n == 0)
      ROS_ERROR_STREAM("kobuki_node : no serial data in.");

    // let packet_finder finds packet
    if (dummy_mode)
    {
      sig_sensor_data.emit();
      sig_wheel_state.emit();
      get_packet = true;
    }

    if (packet_finder.update(buf, n))
    {
      if (serial.remaining() > 28)
      {
        ROS_WARN_STREAM("kobuki_node : serial buffer filling up, clearing [" << serial.remaining() << " bytes]");
        serial.clear(); //is it safe?
      }
      pubtime("packet_find");

      // when packet_finder finds proper packet, we will get the buffer
      packet_finder.getBuffer(data_buffer);

      /*
       static int count=0;
       std::cout << "packet_found: " ;
       std::cout << count++  << " | ";
       std::cout << data_buffer.size() << " | ";
       std::cout << std::endl;
       */

#if 0
      if( verbose )
      {
        printf("Packet: ");
        for( unsigned int i=0; i<data_buffer.size(); i++ )
        {
          printf("%02x ", data_buffer[i] );
          if( i != 0 && ((i%5)==0) ) printf(" ");
        }
      }
#endif
      // deserialise; first three bytes are not data.
      data_buffer.pop_front();
      data_buffer.pop_front();
      data_buffer.pop_front();

      if (protocol_version == "1.0")
      {
        data2.deserialise(data_buffer);
      }
      else if (protocol_version == "2.0")
      {
        sig_index.clear();
        while (data_buffer.size() > 1/*size of etx*/)
        {
          // std::cout << "header_id: " << (unsigned int)data_buffer[0] << " | ";
          // std::cout << "remains: " << data_buffer.size() << " | ";
          switch (data_buffer[0])
          {
            case kobuki_comms::Header::header_default:
              sig_index.insert(data_buffer[0]);
              kobuki_default.deserialise(data_buffer);
              break;
            case kobuki_comms::Header::header_ir:
              sig_index.insert(data_buffer[0]);
              kobuki_ir.deserialise(data_buffer);
              break;
            case kobuki_comms::Header::header_dock_ir:
              sig_index.insert(data_buffer[0]);
              kobuki_dock_ir.deserialise(data_buffer);
              break;
            case kobuki_comms::Header::header_inertia:
              sig_index.insert(data_buffer[0]);
              kobuki_inertia.deserialise(data_buffer);
              break;
            case kobuki_comms::Header::header_cliff:
              sig_index.insert(data_buffer[0]);
              kobuki_cliff.deserialise(data_buffer);
              break;
            case kobuki_comms::Header::header_current:
              sig_index.insert(data_buffer[0]);
              kobuki_current.deserialise(data_buffer);
              break;
            case kobuki_comms::Header::header_magnet:
              sig_index.insert(data_buffer[0]);
              kobuki_magnet.deserialise(data_buffer);
              break;
            case kobuki_comms::Header::header_time:
              sig_index.insert(data_buffer[0]);
              kobuki_time.deserialise(data_buffer);
              break;
            case kobuki_comms::Header::header_hw:
              sig_index.insert(data_buffer[0]);
              kobuki_hw.deserialise(data_buffer);
              break;
            case kobuki_comms::Header::header_fw:
              sig_index.insert(data_buffer[0]);
              kobuki_fw.deserialise(data_buffer);
              break;
            case kobuki_comms::Header::header_st_gyro:
              sig_index.insert(data_buffer[0]);
              kobuki_st_gyro.deserialise(data_buffer);
              break;
            case kobuki_comms::Header::header_eeprom:
              sig_index.insert(data_buffer[0]);
              kobuki_eeprom.deserialise(data_buffer);
              break;
            case kobuki_comms::Header::header_gp_input:
              sig_index.insert(data_buffer[0]);
              kobuki_gp_input.deserialise(data_buffer);
              break;
            default:
              std::cout << "unexpected case reached. flushing current buffer." << std::endl;
              data_buffer.clear();
              break;
          }
          //std::cout << "remains: " << data_buffer.size() << " | ";
          //std::cout << std::endl;
        }
      }
      else
      {
        std::cout << "protocol version is not specified properly." << std::endl;
        return;
        //continue;
      }
      //std::cout << "sig_index_size: " << sig_index.size() << std::endl;
      //ROS_DEBUG_STREAM("kobuki_node:left_encoder [" << data2.data.left_encoder << "], remaining[" << serial.remaining() << "]" );

      //if( verbose ) data.showMe();
      //data.showMe();
      if (protocol_version == "1.0")
      {
        sig_sensor_data.emit();
        sig_wheel_state.emit();
      }
      if (protocol_version == "2.0")
      {
        std::set<unsigned char>::iterator it;
        for (it = sig_index.begin(); it != sig_index.end(); ++it)
        {
          switch ((*it))
          {
            case kobuki_comms::Header::header_default: /*std::cout << " --- " << (int)( *it ) << std::endl;*/
              sig_default.emit();
              sig_sensor_data.emit();
              sig_wheel_state.emit();
              break;
            case kobuki_comms::Header::header_ir: /*std::cout << " --- " << (int)( *it ) << std::endl;*/
              sig_ir.emit();
              break;
            case kobuki_comms::Header::header_dock_ir: /*std::cout << " --- " << (int)( *it ) << std::endl;*/
              sig_dock_ir.emit();
              break;
            case kobuki_comms::Header::header_inertia: /*std::cout << " --- " << (int)( *it ) << std::endl;*/
              sig_inertia.emit();
              break;
            case kobuki_comms::Header::header_cliff: /*std::cout << " --- " << (int)( *it ) << std::endl;*/
              sig_cliff.emit();
              break;
            case kobuki_comms::Header::header_current: /*std::cout << " --- " << (int)( *it ) << std::endl;*/
              sig_current.emit();
              break;
            case kobuki_comms::Header::header_magnet: /*std::cout << " --- " << (int)( *it ) << std::endl;*/
              sig_magnet.emit();
              break;
            case kobuki_comms::Header::header_time: /*std::cout << " --- " << (int)( *it ) << std::endl;*/
              sig_time.emit();
              break;
            case kobuki_comms::Header::header_hw: /*std::cout << " --- " << (int)( *it ) << std::endl;*/
              sig_hw.emit();
              break;
            case kobuki_comms::Header::header_fw: /*std::cout << " --- " << (int)( *it ) << std::endl;*/
              sig_fw.emit();
              break;
            case kobuki_comms::Header::header_st_gyro: /*std::cout << " --- " << (int)( *it ) << std::endl;*/
              sig_st_gyro.emit();
              break;
            case kobuki_comms::Header::header_eeprom: /*std::cout << " --- " << (int)( *it ) << std::endl;*/
              sig_eeprom.emit();
              break;
            case kobuki_comms::Header::header_gp_input: /*std::cout << " --- " << (int)( *it ) << std::endl;*/
              sig_gp_input.emit();
              break;
            default:
              std::cout << "unexpected case reached. flushing current buffer." << std::endl;
              data_buffer.clear();
              break;
          }
        }

      }

      get_packet = true;
      pubtime("packet_emit");
    }

    // send the command packet to mainboard;
    if (get_packet)
      sendCommand();

    // Other threads may have time to do their job.
    // But i still do not know Blcoking mode (without manual usleep) is better way or not.
    // This way is just safe and evaluated for long-time.
    usleep(1250); // at least less then sending period.
  }
}

void Kobuki::getData(kobuki_comms::SensorData &sensor_data)
{
  sensor_data.header0 = data.header0;
  sensor_data.time_stamp = data.time_stamp;
  sensor_data.bump = data.bump;
  sensor_data.wheel_drop = data.wheel_drop;
  sensor_data.cliff = data.cliff;
  sensor_data.left_encoder = data.encoder[0];
  sensor_data.right_encoder = data.encoder[1];
  sensor_data.left_pwm = data.pwm[0];
  sensor_data.right_pwm = data.pwm[1];
  sensor_data.dustbin = data.dustbin;
  sensor_data.remote = data.remote;
  sensor_data.button = data.button;
  sensor_data.charger = data.charger;
  sensor_data.battery = data.battery;
  sensor_data.caster = data.caster;
  sensor_data.over_current = data.over_current;
}

void Kobuki::getData2(kobuki_comms::SensorData &sensor_data)
{
  if (protocol_version == "1.0")
    sensor_data = data2.data;
  if (protocol_version == "2.0")
    sensor_data = kobuki_default.data;
}

void Kobuki::getDefaultData(kobuki_comms::SensorData &sensor_data)
{
  if (protocol_version == "1.0")
    sensor_data = data2.data;
  if (protocol_version == "2.0")
    sensor_data = kobuki_default.data;
}

void Kobuki::getIRData(kobuki_comms::IR &data)
{
  if (protocol_version == "1.0")
    return;
  if (protocol_version == "2.0")
    data = kobuki_ir.data;
}

void Kobuki::getDockIRData(kobuki_comms::DockIR &data)
{
  if (protocol_version == "1.0")
    return;
  if (protocol_version == "2.0")
    data = kobuki_dock_ir.data;
}

void Kobuki::getInertiaData(kobuki_comms::Inertia &data)
{
  if (protocol_version == "1.0")
    return;
  if (protocol_version == "2.0")
    data = kobuki_inertia.data;
}

void Kobuki::getCliffData(kobuki_comms::Cliff &data)
{
  if (protocol_version == "1.0")
    return;
  if (protocol_version == "2.0")
    data = kobuki_cliff.data;
}

void Kobuki::getCurrentData(kobuki_comms::Current &data)
{
  if (protocol_version == "1.0")
    return;
  if (protocol_version == "2.0")
    data = kobuki_current.data;
}

void Kobuki::getMagnetData(kobuki_comms::Magnet &data)
{
  if (protocol_version == "1.0")
    return;
  if (protocol_version == "2.0")
    data = kobuki_magnet.data;
}

void Kobuki::getHWData(kobuki_comms::HW &data)
{
  if (protocol_version == "1.0")
    return;
  if (protocol_version == "2.0")
    data = kobuki_hw.data;
}

void Kobuki::getFWData(kobuki_comms::FW &data)
{
  if (protocol_version == "1.0")
    return;
  if (protocol_version == "2.0")
    data = kobuki_fw.data;
}

void Kobuki::getTimeData(kobuki_comms::Time &data)
{
  if (protocol_version == "1.0")
    return;
  if (protocol_version == "2.0")
    data = kobuki_time.data;
}

void Kobuki::getStGyroData(kobuki_comms::StGyro &data)
{
  if (protocol_version == "1.0")
    return;
  if (protocol_version == "2.0")
    data = kobuki_st_gyro.data;
}

void Kobuki::getEEPROMData(kobuki_comms::EEPROM &data)
{
  if (protocol_version == "1.0")
    return;
  if (protocol_version == "2.0")
    data = kobuki_eeprom.data;
}

void Kobuki::getGpInputData(kobuki_comms::GpInput &data)
{
  if (protocol_version == "1.0")
    return;
  if (protocol_version == "2.0")
    data = kobuki_gp_input.data;
}

void Kobuki::getJointState(device_comms::JointState &joint_state)
{
  static bool init_l = false;
  static bool init_r = false;
  double diff_ticks = 0.0f;
  unsigned short curr_tick_left = 0;
  unsigned short curr_tick_right = 0;
  unsigned short curr_timestamp = 0;

  if (joint_state.name == "wheel_left")
  {
    if (protocol_version == "1.0")
    {
      curr_tick_left = data2.data.left_encoder;
      curr_timestamp = data2.data.time_stamp;
    }
    if (protocol_version == "2.0")
    {
      curr_tick_left = kobuki_default.data.left_encoder;
      curr_timestamp = kobuki_default.data.time_stamp;
    }
    if (!init_l)
    {
      last_tick_left = curr_tick_left;
      init_l = true;
    }
    diff_ticks = (double)(short)((curr_tick_left - last_tick_left) & 0xffff);
    last_tick_left = curr_tick_left;
    last_rad_left += tick_to_rad * diff_ticks;
    last_mm_left += tick_to_mm / 1000.0f * diff_ticks;
    joint_state.position = last_rad_left;
    joint_state.velocity = last_mm_left;
  }
  else // wheel_right
  {
    if (protocol_version == "1.0")
    {
      curr_tick_right = data2.data.right_encoder;
      curr_timestamp = data2.data.time_stamp;
    }
    if (protocol_version == "2.0")
    {
      curr_tick_right = kobuki_default.data.right_encoder;
      curr_timestamp = kobuki_default.data.time_stamp;
    }

    if (!init_r)
    {
      last_tick_right = curr_tick_right;
      init_r = true;
    }
    diff_ticks = (double)(short)((curr_tick_right - last_tick_right) & 0xffff);
    last_tick_right = curr_tick_right;
    last_rad_right += tick_to_rad * diff_ticks;
    last_mm_right += tick_to_mm / 1000.0f * diff_ticks;
    joint_state.position = last_rad_right;
    joint_state.velocity = last_mm_right;
  }

  if (curr_timestamp != last_timestamp)
  {
    last_diff_time = ((double)(short)((curr_timestamp - last_timestamp) & 0xffff)) / 1000.0f;
    last_timestamp = curr_timestamp;
  }
  joint_state.velocity = (tick_to_rad * diff_ticks) / last_diff_time;

  joint_state.enabled = is_connected && is_running && is_enabled;

#if 0
  std::cout << joint_state.name
  << "[" << diff_ticks << "]"
  << "[" << last_rad_left << "]"
  << "[" << last_mm_left << "]"
  << std::endl;
#endif
  //joint_state.velocity=0.0f;

  //are there considerations of overflow of position?
  //are there range or limits?
  //is it signed or unsigned?

  //ticks [0-65535]
  //positions [?-?]
  //double [], store last data and integrate here.
}

void Kobuki::setCommand(double vx, double wz)
{
  if (wz == 0.0f)
    radius = 0;
  else if (vx == 0.0f && wz > 0.0f)
    radius = 1;
  else if (vx == 0.0f && wz < 0.0f)
    radius = -1;
  else
    radius = (short)(vx * 1000.0f / wz);

  speed = (short)(1000.0f * std::max(vx + bias * wz / 2.0f, vx - bias * wz / 2.0f));
}

void Kobuki::sendCommand()
{
  //std::cout << "speed = " << speed << ", radius = " << radius << std::endl;
  unsigned char cmd[] = {0xaa, 0x55, 5, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
  unsigned char cs(0);

  union_sint16 union_speed, union_radius;
  union_speed.word = speed;
  union_radius.word = radius;

  cmd[4] = union_speed.byte[0];
  cmd[5] = union_speed.byte[1];
  cmd[6] = union_radius.byte[0];
  cmd[7] = union_radius.byte[1];

  //memcpy(cmd + 4, &speed,  sizeof(short) );
  //memcpy(cmd + 6, &radius, sizeof(short) );

  for (int i = 2; i <= 6; i++)
    cs ^= cmd[i];
  cmd[8] = cs;

  serial.write(cmd, 9);
  pubtime("send_cmd");
}

void Kobuki::sendCommand(const kobuki_comms::CommandConstPtr &data)
{
  kobuki_command.data = *data;

  command_buffer.clear();
  command_buffer.resize(64);
  command_buffer.push_back(0xaa);
  command_buffer.push_back(0x55);
  command_buffer.push_back(0); // size of payload only, not stx, not etx, not length

  if (!kobuki_command.serialise(command_buffer))
  {
    ROS_ERROR_STREAM("command serialise failed.");
  }

  command_buffer[2] = command_buffer.size() - 3;
  unsigned char checksum = 0;
  for (unsigned int i = 2; i < command_buffer.size(); i++)
    checksum ^= (command_buffer[i]);

  command_buffer.push_back(checksum);
  serial.write(&command_buffer[0], command_buffer.size());

  for (unsigned int i = 0; i < command_buffer.size(); ++i)
  {
    std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') << (unsigned)command_buffer[i] << std::dec
        << std::setfill(' ') << " ";
  }

  std::cout << std::endl;

  if (kobuki_command.data.command == kobuki_comms::Command::commandBaseControl)
  {
    radius = kobuki_command.data.radius;
    speed = kobuki_command.data.speed;
  }
}

bool Kobuki::run()
{
//	is_running = true;
  is_enabled = true;
  return true;
}

bool Kobuki::stop()
{
  setCommand(0.0f, 0.0f);
  sendCommand();
//	is_running = false;
  is_enabled = false;
  return true;
}

void Kobuki::pubtime(const char *str)
{
  return;
  //if( str != "every_tick" ) return ;
  TimeStamp time = stopwatch.split();

  //ROS_INFO_STREAM( "ecl_time:stopwatch:" << str << ":[" << time.sec() + time.usec()/0.000001 << "s]");

  std::stringstream s;
  s.precision(6);
  s << "ecl_time:stopwatch:" << str << ":[" << time.sec() + time.usec() * 0.000001 << "s]";
  ROS_INFO_STREAM(s.str());

  return;
}

} // namespace kobuki
