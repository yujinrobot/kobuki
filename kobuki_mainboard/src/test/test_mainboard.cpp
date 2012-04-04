/**
 * @file /iclebo_mainboard/src/test/test_mainboard.cpp
 *
 * @brief test the interface to iclebo mainboard
 *
 * @date October 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include <iostream>

#include "../../include/iclebo_mainboard/iclebo_mainboard.hpp"

#include "sound_manager.hpp"
#include "display_manager.hpp"


/*****************************************************************************
** Main
*****************************************************************************/

void test_motors()
{
	std::cout << "hello YOUNG HOON JOO Start packet finder test!" << std::endl;

	iCleboMainboardDriver iclebo_receiver( "/dev/ttyUSB0", true );
	iclebo_receiver.start();

	short int test_speed(0);
	bool forward(true);

	iclebo_receiver.cc_base.radius = 0;
	iclebo_receiver.cc_base.speed = 0;
	iclebo_receiver.cc_base.yes = true;

	iclebo_receiver.cc_cleaning_motors.brush_power = 0;
	iclebo_receiver.cc_cleaning_motors.vacuum_power = 0;
	iclebo_receiver.cc_cleaning_motors.side_brush_power = 0;
	iclebo_receiver.cc_cleaning_motors.yes = true;

	while( iclebo_receiver.isRunning() )
	{

		if( forward && ++ test_speed >= 300 ) forward = false;
		else if( !forward && --test_speed <= -300 ) forward = true;

		usleep(100000);

		iclebo_receiver.cc_cleaning_motors.brush_power 			= 40;		// 20 %
		iclebo_receiver.cc_cleaning_motors.vacuum_power 		= 40;		// 45 %
		iclebo_receiver.cc_cleaning_motors.side_brush_power 	= 15;	// 15 %
		iclebo_receiver.cc_cleaning_motors.yes = true;

		iclebo_receiver.cc_base.radius = 0;
		iclebo_receiver.cc_base.speed = test_speed;
		iclebo_receiver.cc_base.yes = false;
	}
}


void test_display()
{
	std::cout << "hello YOUNG HOON JOO Start packet finder test!" << std::endl;

	iCleboMainboardDriver iclebo_receiver( "/dev/ttyUSB0", true );
	iclebo_receiver.start();

	iclebo_receiver.cc_base.radius = 0;
	iclebo_receiver.cc_base.speed = 0;
	iclebo_receiver.cc_base.yes = true;

	iclebo_receiver.cc_cleaning_motors.brush_power = 0;
	iclebo_receiver.cc_cleaning_motors.vacuum_power = 0;
	iclebo_receiver.cc_cleaning_motors.side_brush_power = 0;
	iclebo_receiver.cc_cleaning_motors.yes = true;

	while( iclebo_receiver.isRunning() )
	{
		for( unsigned int i(0); i<8; i++ )
		{
			iclebo_receiver.cc_display.digits[i] = 0xff;
			iclebo_receiver.cc_display.intensities[i] ++;
		}
		iclebo_receiver.cc_display.yes = true;
		usleep(100000);
	}
}


void test_burning()
{
	std::cout << "hello YOUNG HOON JOO Start packet finder test!" << std::endl;

	iCleboMainboardDriver iclebo_receiver( "/dev/ttyUSB0", true );
	iclebo_receiver.start();

	int target_sound(0);
	soundManager sound_manager;
	sound_manager.loadSound("sound.txt");
	sound_manager.showMeSound();
	sound_manager.setSound( target_sound );

	displayManager display_manager;

	short int test_speed(0);
	bool forward(true);

	iclebo_receiver.cc_base.radius = 0;
	iclebo_receiver.cc_base.speed = 0;
	iclebo_receiver.cc_base.yes = true;

	iclebo_receiver.cc_cleaning_motors.brush_power = 0;
	iclebo_receiver.cc_cleaning_motors.vacuum_power = 0;
	iclebo_receiver.cc_cleaning_motors.side_brush_power = 0;
	iclebo_receiver.cc_cleaning_motors.yes = true;

	iclebo_receiver.cc_sound.note = 1000;
	iclebo_receiver.cc_sound.duration = 10;
	iclebo_receiver.cc_sound.yes = true;

	while( iclebo_receiver.isRunning() )
	{

		if( forward && ++ test_speed >= 300 ) forward = false;
		else if( !forward && --test_speed <= -300 ) forward = true;

		usleep(20000);

		iclebo_receiver.cc_cleaning_motors.brush_power = 40;		// 20 %
		iclebo_receiver.cc_cleaning_motors.vacuum_power = 40;		// 45 %
		iclebo_receiver.cc_cleaning_motors.side_brush_power = 0;	// 15 %
		iclebo_receiver.cc_cleaning_motors.yes = true;

		iclebo_receiver.cc_base.radius = 0;
		iclebo_receiver.cc_base.speed = test_speed;
		iclebo_receiver.cc_base.yes = true;

		if( sound_manager.playSound( iclebo_receiver.cc_sound.note,
									 iclebo_receiver.cc_sound.duration ) )
		{
			iclebo_receiver.cc_sound.duration /= 10;
			iclebo_receiver.cc_sound.duration += 10;
			iclebo_receiver.cc_sound.yes = true;
		}
		else
		{
			iclebo_receiver.cc_sound.yes = false;
			usleep(1000000);
			sound_manager.setSound( ++target_sound );
		}

		display_manager.setText( ' ', 'A', 'b', 'c' );
		display_manager.setValue( 1234 );
		display_manager.setBattery( 60 );
		display_manager.setIcons( false, false, false, true );
		display_manager.setIcons( 3 );
		display_manager.setTime( 15, 34 );
		display_manager.run( iclebo_receiver.cc_display.digits,
									  iclebo_receiver.cc_display.intensities );

		iclebo_receiver.cc_display.yes = true;
	}
}



int main(int argc, char **argv)
{

	test_motors();
//	test_display();
//	test_burning();

	return 0;
}

