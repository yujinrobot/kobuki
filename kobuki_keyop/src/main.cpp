/**
 * @file /kobuki_keyop/src/main.cpp
 *
 * @brief Executable code for the keyop node.
 *
 * @date October 2010.
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/keyop_core/keyop_core.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using keyop_core::KeyOpCore;

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

	ros::init(argc, argv, "kobuki_keyop");

	KeyOpCore keyop;
	keyop.init();
	keyop.spin();
	std::cout << "Program exiting" << std::endl;
	return 0;
}
