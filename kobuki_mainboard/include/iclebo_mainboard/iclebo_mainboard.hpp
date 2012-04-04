#ifndef ICLEBO_MAINBOARD_HPP_
#define ICLEBO_MAINBOARD_HPP_


#include <ecl/threads.hpp>
#include <ecl/devices.hpp>
#include <ecl/time.hpp>

#include <packet_handler/packet_finder.hpp>
#include "iclebo_mainboard_data.hpp"
#include "iclebo_mainboard_command.hpp"

// this is just test purpose things do not use it

using ecl::Threadable;
using ecl::Serial;

class icleboPacketFinder : public packet_handler::packetFinder
{
public:
	bool checkSum()
	{
		unsigned int packet_size( buffer.size() );
		unsigned char cs(0);
		for( unsigned int i=2; i<packet_size; i++ )
		{
			cs ^= buffer[i];
		}

		if( cs )
		{
			for( unsigned int i(0); i< packet_size; i++ )
			{
				printf("%02x ", buffer[i] );
			}
			std::cout << " [CheckSum error]" << std::endl;
		}
		return cs ? false : true;
	}
};


class iCleboMainboardDriver : public Threadable
{
public:

	iCleboMainboardDriver( const std::string & portName, bool verboseEnable=false )
	:
	serial( portName, ecl::BaudRate_115200, ecl::DataBits_8, ecl::StopBits_1, ecl::NoParity ),
	get_out(false),
	verbose(verboseEnable),
	buffer_to_send(100)
	{
		ecl::PushAndPop<unsigned char> stx(2,0);
		ecl::PushAndPop<unsigned char> etx(1);
		stx.push_back( 0xaa );
		stx.push_back( 0x55 );
		packet_finder.configure( stx, etx, 1, 64, 1, true );
//		packet_finder.enableVerbose();
	}
	~iCleboMainboardDriver() { get_out = true; }

	void runnable()
	{
		unsigned char buf[256];
		bool get_packet;

		while( !get_out )
		{
			// Now, we have not packet
			get_packet = false;

			// get the byte(s) from the serial port
			int n( serial.read( buf, packet_finder.numberOfDataToRead() ) );


//			for( int i(0); i<n; i++ )
//				printf( "%02x ", buf[i] );
//			if( n > 0 ) printf("\n");

			// let packet_finder finds packet
			if( packet_finder.update( buf, n ) )
			{
				// yes we got the packet now
				get_packet = true;

				// when packet_finder finds proper packet, we will get the buffer
				packet_finder.getBuffer( data_buffer );

				if( verbose )
				{
					printf("Packet: ");
					for( unsigned int i=0; i<data_buffer.size(); i++ )
					{
						printf("%02x ", data_buffer[i] );
						if( i != 0 && ((i%5)==0) ) printf(" ");
					}
				}

				// deserialise; first three bytes are not data.
				data_buffer.pop_front();
				data_buffer.pop_front();
				data_buffer.pop_front();
				data.deserialise( data_buffer );

				if( verbose ) data.showMe();
			}

			// send the command packet to mainboard;
			if( get_packet ) sendCommand();

			// Other threads may have time to do their job.
			// But i still do not know Blcoking mode (without manual usleep) is better way or not.
			// This way is just safe and evaluated for long-time.
			usleep(5000);
		}
	}

	/**
	 * This driver just provides setting base velocity.
	 * todo; It will be extended to generic form like as iclebo_mainboard_data
	 *       which support serialisation and deserialisation.
	 */
	void sendCommand()
	{
		//clear buffer
		buffer_to_send.clear();

		// first signature
		buffer_to_send.push_back( 0xaa );

		// second signature
		buffer_to_send.push_back( 0x55 );

		// number of data to send
		buffer_to_send.push_back( 0x00 );

		// put the data according to the state of commands
		if( cc_base.yes )
		{
			cc_base.serialise( buffer_to_send );
			cc_base.yes = false;
		}
		if( cc_sound.yes )
		{
			cc_sound.yes = false;
			cc_sound.serialise( buffer_to_send );
		}
		if( cc_display.yes )
		{
			cc_display.yes = false;
			cc_display.serialise( buffer_to_send );
		}
		if( cc_cleaning_motors.yes )
		{
			cc_cleaning_motors.yes = false;
			cc_cleaning_motors.serialise( buffer_to_send );
		}

		if( buffer_to_send.size() <= 3 ) return;

		// fill length field
		buffer_to_send[2] = buffer_to_send.size() - 3;

		// fill the check sum field
		unsigned char cs(0);
		for( unsigned int i=2; i< buffer_to_send.size(); i++  )
			cs ^= buffer_to_send[i];

		buffer_to_send.push_back( cs );

		serial.write( &buffer_to_send[0], buffer_to_send.size() );

//		std::cout << "send the data: ";
//		for( unsigned int i=0; i<buffer_to_send.size(); i++ )
//		{
//			printf("%02x ", buffer_to_send[i] );
//			if( i%5 == 0 )printf(" ");
//		}
//		std::cout << std::endl;
	}

	void getData( icleboMainboardData & newData )
	{
		newData = data;
	}

//protected:
	Serial serial;
	bool get_out;
	bool verbose;
	icleboMainboardData data;
	icleboPacketFinder packet_finder;
	icleboPacketFinder::BufferType data_buffer;

	ecl::PushAndPop<unsigned char> buffer_to_send;
	ccBase 			cc_base;
	ccSound 		cc_sound;
	ccDisplay 		cc_display;
	ccCleaningMotor cc_cleaning_motors;
};


#endif /* ICLEBO2_HPP_ */
