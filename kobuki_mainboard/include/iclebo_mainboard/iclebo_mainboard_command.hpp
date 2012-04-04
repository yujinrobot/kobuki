
#ifndef ICLEBO_MAINBOARD_COMMAND_HPP_
#define ICLEBO_MAINBOARD_COMMAND_HPP_

#include <ecl/containers.hpp>
#include <packet_handler/payload_base.hpp>


enum commandList
{
  commandBaseControl 	= 0x01,
  commandCleaningMotor	= 0x02,
  commandSound			= 0x03,
  commandSoundSequence	= 0x04,
  commandAnimation		= 0x05,
  commandDisplay		= 0x06,

  commandListSize
};

class ccBase : public packet_handler::payloadBase
{
public:
	ccBase()
	: packet_handler::payloadBase(), code( static_cast<unsigned char>(commandBaseControl) )
	{}

	bool serialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		buildBytes( code, byteStream );
		buildBytes( speed, byteStream );
		buildBytes( radius, byteStream );

		return true;
	}

	bool deserialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		buildVariable( code, byteStream );
		buildVariable( speed, byteStream );
		buildVariable( radius, byteStream );

		yes = constrain();
		return yes;
	}

	bool constrain()
	{
		if( abs(speed) > 600 ) return false;
		return true;
	}

	unsigned char code;			// command id, please refer to enum commandList
	signed short int speed;		// desired translational speed [mm]
	signed short int radius;	// desired radius [mm]
};


class ccCleaningMotor : public packet_handler::payloadBase
{
public:
	ccCleaningMotor()
	:packet_handler::payloadBase(), code( static_cast<unsigned char>(commandCleaningMotor) )
	{}

	bool serialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		buildBytes( code, byteStream );
		buildBytes( brush_power, byteStream );
		buildBytes( vacuum_power, byteStream );
		buildBytes( side_brush_power, byteStream );

		return true;
	}

	bool deserialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		buildVariable( code, byteStream );
		buildVariable( brush_power, byteStream );
		buildVariable( vacuum_power, byteStream );
		buildVariable( side_brush_power, byteStream );

		yes = constrain();
		return yes;
	}

	bool constrain()
	{
		if( brush_power > 100 ) 		return false;
		if( vacuum_power > 100 ) 		return false;
		if( side_brush_power > 100 ) 	return false;
		return true;
	}

	unsigned char code;					// command id, please refer to enum commandList
	unsigned char brush_power;			// desired power [%] for brush motor
	unsigned char vacuum_power;			// desired power [%] for vacuum motor
	unsigned char side_brush_power;		// desired power [%] for side brush motor
};

class ccSound : public packet_handler::payloadBase
{
public:
	ccSound()
	: packet_handler::payloadBase(), code( static_cast<unsigned char>(commandSound) ),
	 note(0),
	 duration(0)
	{}

	bool serialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		buildBytes( code, byteStream );
		buildBytes( note, byteStream );
		buildBytes( duration, byteStream );

		return true;
	}

	bool deserialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		buildVariable( code, byteStream );
		buildVariable( note, byteStream );
		buildVariable( duration, byteStream );

		yes = constrain();
		return yes;
	}

	bool constrain()
	{
		if( note  > 22017 || note < 72 ) 		return false;
		if( duration == 0 || duration > 250 ) 	return false;
		return true;
	}

	unsigned char 		code;					// command id, please refer to enum commandList
	unsigned short int 	note;					// note defined in fw; please refer to protocol document
	unsigned char		duration;				// duration for play [10msec]
};

class ccDisplay : public packet_handler::payloadBase
{
public:
	ccDisplay()
	:packet_handler::payloadBase(), code( static_cast<unsigned char>(commandDisplay) )
	{
		for( unsigned int i(0); i<8; i++ )
		{
			intensities[i] = 0;
			digits[i] = 0;
		}
	}

	bool serialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		buildBytes( code, byteStream );
		for( int i=0; i<7; i++ )
			buildBytes( digits[i], byteStream );

		for( int i(0); i<4; i++ )
		{
			unsigned char intensity;
			intensity  = intensities[2*i+0] & 0x0f;
			intensity |= ((intensities[2*i+1]<<4) & 0xf0);
			buildBytes( intensity, byteStream );
		}

		return true;
	}

	bool deserialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		buildVariable( code, byteStream );

		yes = constrain();
		return yes;
	}

	bool constrain()
	{
		return true;
	}

	unsigned char 	code;			// command id, please refer to enum commandList
	unsigned char 	digits[8];		// define the segment's code; please refer to document
	unsigned char 	intensities[8];	// bit array; please refer to protocol document to know the bit order for each icon
};


#endif /* ROBOT_DATA_HPP_ */
