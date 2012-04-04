/*
 * display_manager.hpp
 *
 *  Created on: Apr 13, 2011
 *      Author: jakan2
 */

#ifndef DISPLAY_MANAGER_HPP_
#define DISPLAY_MANAGER_HPP_



class sevSegmentStandard
{
public:
	sevSegmentStandard()
	{
	}
	unsigned char operator() ( char ch )
	{
		if( ch & 0x80 )
		{
			return (ch & 0x7f);
		}

		switch(ch)
		{
			// 0~15
			case 0:			return 0x7e;
			case 1: 		return 0x30;
			case 2: 		return 0x6d;
			case 3: 		return 0x79;
			case 4: 		return 0x33;
			case 5: 		return 0x5b;
			case 6: 		return 0x5f;
			case 7: 		return 0x72;
			case 8: 		return 0x7f;
			case 9: 		return 0x73;
			case 10: 		return 0x77;
			case 11: 		return 0x1f;
			case 12: 		return 0x0d;
			case 13: 		return 0x3d;
			case 14: 		return 0x4f;
			case 15: 		return 0x47;

			// characters
			case 'b':		return 0x1f;
			case 'r':		return 0x05;
			case 'v': 		return 0x1c;
			case 'A':		return 0x77;
			case 'C':		return 0x4e;
			case 'I':		return 0x30;
			case 'E':		return 0x4f;
			case 'U':		return 0x3e;
			case 'L':		return 0x0e;
			case 'd':		return 0x3d;
			case 'o':		return 0x1d;
			case 'c':		return 0x0d;
			case 't':		return 0x0f;
			case 'n':		return 0x15;
			case 'H':		return 0x37;
			case 'F':		return 0x47;
			case 'S':		return 0x5b;
			case 'P':		return 0x67;

			// default is zero; erase
			default: 		return 0;
		}
	}
};


class displayManager
{
public:
	displayManager( const unsigned int samplingTime=20 )
	: sampling_time(samplingTime),
	  time_mode(false),
	  time_counter(0)
	{
		removeAll();
	}

	~displayManager()
	{}

	void removeAll()
	{
		for( int i=0; i<8; i++ )
		{
			digits[i] 		= 0;
			intensities[i] 	= 0;
		}
	}

	void setText( char d0, char d1, char d2, char d3, int i0=15, int i1=15, int i2=15, int i3=15 )
	{
		digits[0] = seg_cov( d3 );
		digits[1] = seg_cov( d2 );
		digits[2] = seg_cov( d1 );
		digits[3] = seg_cov( d0 );

		intensities[0] = i3;
		intensities[1] = i2;
		intensities[2] = i1;
		intensities[3] = i0;
	}

	void setValue( const int value )
	{
		// set the segment and intensity
		int d[4];

		d[0] = value/1000;
		d[1] = (value - d[0]*1000)/100;
		d[2] = (value - d[0]*1000 - d[1]*100)/10;
		d[3] = (value - d[0]*1000 - d[1]*100 - d[2]*10);

		setText( d[0]==0?' ':d[0],
				 d[1]==0?' ':d[1],
				 d[2]==0?' ':d[2],
				 d[3] );
	}

	/**
	 *
	 * @param value 0 ~ 100%
	 */
	void setBattery( const int batteryValue )
	{
		int value( batteryValue );
		if( value < 10 ) value = 10;
		else if( value > 100 ) value = 100;

		digits[5] 		= 0;
		intensities[5] 	= 15;

		if( value >= 70 ) 		digits[5] = 7<<4;
		else if( value >= 30 ) 	digits[5] = 3<<4;
		else if( value >= 10 ) 	digits[5] = 1<<4;
	}

	void run( unsigned char * digitData, unsigned char * intensityData )
	{
		for( int i=0; i<8; i++ )
		{
			digitData[i] = digits[i];
			intensityData[i] = intensities[i];
		}

		play_time += sampling_time;

		if( time_mode )
		{
			// toggle dot
			time_counter += sampling_time;
			if( time_counter >= 500 )
			{
				time_counter = 0;
				digits[4] ^= (0x10);
			}
		}
	}

	void setIcons( bool m3, bool m2, bool m1, bool m0 )
	{
		digits[6]  = m3 ? 0x08 : 0;
		digits[6] |= m2 ? 0x10 : 0;
		digits[6] |= m1 ? 0x20 : 0;
		digits[6] |= m0 ? 0x40 : 0;

		intensities[6] 	= 15;
	}

	/**
	 *
	 * @param icon: 0: reservation
	 *         		1: mopping
	 *         		2: full mode
	 *         		3: zigzag
	 *
	 */
	void setIcons( int mode )
	{
		switch( mode )
		{
		case 3:	setIcons( true, false, false, false );	break;
		case 2:	setIcons( false, true, false, false );	break;
		case 1:	setIcons( false, false, true, false );	break;
		case 0:	setIcons( false, false, false, true );	break;
		default:										break;
		}
	}


	void setTime( int hour, int min )
	{
		setValue( hour*100 + min );
		if( !time_mode ) time_counter = 0;
		time_mode = true;
		intensities[4] 	= 15;
	}

	void disableTimeMode()
	{
		time_mode = false;
		digits[4] = 0;
	}

private:
	unsigned char digits[8];
	unsigned char intensities[8];
	sevSegmentStandard seg_cov;

	unsigned int sampling_time;			// [msec]
	unsigned int play_time;

	unsigned int time_counter;
	bool time_mode;
};


#endif /* DISPLAY_MANAGER_HPP_ */
