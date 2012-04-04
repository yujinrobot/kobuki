/*
 * sound_manager.hpp
 *
 * In order to control the sound block of iclebo second version.
 * I'd like to use the some stl components, however our target system may not have good
 * stl implementation. So that i had to use the c-like implementation.
 *
 * Sound file format
 *
 * <number of sounds>								// maximum number of sound are 32
 * <sound name=integer_value> <number of lines>		// for 1st sound, maximum number of lines are 32
 * <note> <duration>
 * .
 * .
 * .
 * .
 * <note> <duration>
 * <sound name=integer_value> <number of lines>		// for 2nd sound
 * <note> <duration>
 * .
 * .
 * .
 * .
 * <note> <duration>
 *
 * note is frequency [Hz], It will be converted to tick value for iclebo first timer block
 * duration is milli - sec. It will be converted to 10msec when below soundManager send this duration to iclebo
 * maximum duration is just 2550 msec
 *
 *
 *
 *  Created on: Apr 13, 2011
 *      Author: jakan2
 */

#ifndef SOUND_MANAGER_HPP_
#define SOUND_MANAGER_HPP_


const int invalid_note = 0xffff;
const int max_sounds 	= 32;
const int max_lines		= 32;

class oneNote
{
public:
	oneNote()
	: note(invalid_note), duration(invalid_note)
	{}

	unsigned short int	note;			// tick value for iclebo. This value will be converted from frquency [Hz]
	unsigned short int	duration;		// msec value for iclebo. This value will be converted from milli-second
};

class soundManager
{
public:
	soundManager( const double & HzToTick = 359971.202303816, const int samplingTime=20 )
	: num_sounds(0),
	  verbose(true),
	  hz2tick( HzToTick),
	  sampling_time(20),
	  enable(false)
	{}

	~soundManager()
	{}

	/**
	 * load the sound file from the file system
	 * @param fileName: this function will open the file with this parameter.
	 * @return	-1: could not open the file
	 * 			-2: exceeds maximum sounds
	 * 			-3: exceeds number of lines
	 *
	 */
	int loadSound( char * fileName )
	{
		int ret;
		int num;
		FILE * pf = fopen( fileName, "r" );
		if( pf == 0 ) return -1;

		// load number of sound
		ret = fscanf( pf, "%d\n", &num );
		if( verbose )
		{
			std::cout << "Number of sounds are " << num << std::endl;
		}

		if( num > max_sounds )
		{
			fclose(pf);
			return -2;
		}

		// load each sound
		int sound_name(0);
		int num_lines(0);
		int note;
		int duration;
		for( int i(0); i<num; i++ )
		{
			ret = fscanf( pf, "%d %d\n", &sound_name, &num_lines );
			if( num_lines > max_lines )
			{
				if( verbose ) std::cout << "Exceeds number of lines[" << num_lines << "]" << std::endl;
				fclose(pf);
				return -3;
			}

			for( int j(0); j<num_lines; j++ )
			{
				ret = fscanf( pf, "%d %d\n", &note, &duration );
				sound[i][j].note = static_cast<unsigned short int>(1.0 / static_cast<double>(note) * hz2tick);
				sound[i][j].duration = duration;
			}
			sound[i][num_lines].note = invalid_note;

		}

		num_sounds = num;

		fclose(pf);

		return 0;
	}

	void showMeSound()
	{
		if( verbose )
		{
			std::cout << "Number of sounds are " << num_sounds << std::endl;
		}

		for( int i=0; i<num_sounds; i++ )
		{
			std::cout << "Sound: " << i << std::endl;

			for( int j=0; j<max_lines; j++ )
			{
				if( sound[i][j].note == invalid_note ) break;
				printf("%07d   %03d \n", sound[i][j].note, sound[i][j].duration );
			}
		}
	}

	int setSound( const int soundName )
	{
		if( soundName < 0 || soundName >= num_sounds  ) return -1;
		target_sound = &(sound[soundName][0]);
		play_time = 0;
		enable = true;
		return 0;
	}

	/**
	 *
	 * @param note 			tick for iclebo
	 * @param duration		msec
	 * @return
	 */
	bool playSound( unsigned short int & note, unsigned char & duration )
	{
		if( !enable ) return false;
		if( play_time > target_sound->duration )
		{
			target_sound ++;
			play_time = 0;
			if( target_sound->note == invalid_note )
			{
				note = 0;
				duration = 0;
				enable = false;
				return false;
			}
		}

		note 		= target_sound->note;
		duration 	= target_sound->duration;
		play_time 	+= sampling_time;

		return true;
	}


	int num_sounds;
	oneNote sound[max_sounds][max_lines];
	bool verbose;
	double hz2tick;
	int sampling_time;	//[msec]
	int play_time;		//[msec]
	oneNote * target_sound;
	bool enable;
};


#endif /* SOUND_MANAGER_HPP_ */
