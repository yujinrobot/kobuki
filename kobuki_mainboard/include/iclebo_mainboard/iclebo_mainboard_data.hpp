
#ifndef ICLEBO_MAINBOARD_DATA_HPP_
#define ICLEBO_MAINBOARD_DATA_HPP_

#include <ecl/containers.hpp>
#include <packet_handler/payload_base.hpp>


class icleboMainboardData : public packet_handler::payloadBase
{
public:
	// group 0
	unsigned char header0;
	unsigned short int time_stamp;
	unsigned char bump;
	unsigned char wheel_drop;
	unsigned char cliff;
	unsigned short int encoder[2];	// left and right
	unsigned char pwm[2];			// left and right
	unsigned char dustbin;			// 1:locked 0:unlocked
	unsigned char remote;
	unsigned char button;
	unsigned char charger; 			//
	unsigned char battery;			// [mA/10]
	unsigned char caster;			// tick
	unsigned char over_current;		// wheel left/right, brush, side-brush, vacuum

	// group 1
	unsigned char header1;
	unsigned char obstacle[3];
	unsigned char docking[3];

	unsigned char header2;

	// group 4 - loaded current for each motor. This is temporal setting until i design complete feedback packet
	unsigned char header4;
	unsigned char current[6];

	unsigned char header5;
	signed short int angle;

	unsigned short int bottom[3];
	int x, y, t;

	/**
	 *
	 * @param byteStream
	 * @return
	 */
	bool serialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		buildBytes( header0, 	byteStream );
		buildBytes( time_stamp, 	byteStream );
		buildBytes( bump, 		byteStream );
		buildBytes( wheel_drop, 	byteStream );
		buildBytes( cliff, 		byteStream );
		buildBytes( encoder[0], 	byteStream );
		buildBytes( encoder[1], 	byteStream );
		buildBytes( pwm[0], 		byteStream );
		buildBytes( pwm[1], 		byteStream );
		buildBytes( dustbin, 	byteStream );
		buildBytes( remote, 		byteStream );
		//buildBytes( button, 	byteStream );
		buildBytes( charger, 	byteStream );
		buildBytes( battery, 	byteStream );
		buildBytes( caster, 		byteStream );
		buildBytes( over_current,byteStream );

		buildBytes( header1, 	byteStream );
		buildBytes( obstacle[0], byteStream );
		buildBytes( obstacle[1], byteStream );
		buildBytes( obstacle[2], byteStream );
		buildBytes( docking[3], 	byteStream );

		return true;
	}
	bool deserialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		buildVariable( header0, 	byteStream );
		buildVariable( time_stamp, 	byteStream );
		buildVariable( bump, 		byteStream );
		buildVariable( wheel_drop, 	byteStream );
		buildVariable( cliff, 		byteStream );
		buildVariable( encoder[0], 	byteStream );
		buildVariable( encoder[1], 	byteStream );
		buildVariable( pwm[0], 		byteStream );
		buildVariable( pwm[1], 		byteStream );
		buildVariable( dustbin, 	byteStream );
		buildVariable( remote, 		byteStream );
//		buildVariable( button, 		byteStream );
		buildVariable( charger, 	byteStream );
		buildVariable( battery, 	byteStream );
		buildVariable( caster, 		byteStream );
		buildVariable( over_current,byteStream );

		buildVariable( header1, 	byteStream );
		buildVariable( obstacle[0], byteStream );
		buildVariable( obstacle[1], byteStream );
		buildVariable( obstacle[2], byteStream );

		buildVariable( header2, 	byteStream );
		buildVariable( docking[0], 	byteStream );
		//buildVariable( docking[1], 	byteStream );
		//buildVariable( docking[2], 	byteStream );

//		buildVariable( x, 	byteStream );
//		buildVariable( y, 	byteStream );
//		buildVariable( t, 	byteStream );

//		buildVariable( header4, 	byteStream );
//		for( unsigned int i(0); i<6; i++ )
//		{
//			buildVariable( current[i], byteStream );
//		}
//		buildVariable( header5, 	byteStream );
//		buildVariable( angle, 	byteStream );
//		buildVariable( bottom[0], 	byteStream );
//		buildVariable( bottom[1], 	byteStream );
//		buildVariable( bottom[2], 	byteStream );
		return constrain();
	}

	bool constrain()
	{
		return true;
	}

	void showMe()
	{
		//printf("--[%02x || %03d | %03d | %03d] || [%+4d] \n", bump, obstacle[2], obstacle[1], obstacle[0], angle );
		printf("--");
//		for( int i(0); i<6; i++ )
//		{
//			printf("[%06d]", current[i]*10 );
//		}
		printf("{%05d}", bottom[2] );
		printf("{%05d}", bottom[1] );
		printf("{%05d}", bottom[0] );
		printf("\n");
		double psd_b( static_cast<double>(encoder[0]) );
		double psd_a( static_cast<double>(encoder[1]) );

		psd_a *= (4.0/3.0);
		psd_b *= (4.0/3.0);

		double r((psd_a-psd_b)/(psd_a+psd_b));
		//printf("psd[%05d][%05d]--> %1.2f \n", encoder[0], encoder[1], -31.71*r*r*r + 35.149*r*r-46.981*r+57.312 );
	}
};

#endif /* ROBOT_DATA_HPP_ */
