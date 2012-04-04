/*
 *	Copyright (C) 2011 by Pandora Robotics Team, Aristotle Univeristy of Thessaloniki, Greece
 *
 *	Permission is hereby granted, free of charge, to any person obtaining a copy
 *	of this software and associated documentation files (the "Software"), to deal
 *	in the Software without restriction, including without limitation the rights
 *	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *	copies of the Software, and to permit persons to whom the Software is
 *	furnished to do so, subject to the following conditions:
 *
 *	The above copyright notice and this permission notice shall be included in
 *	all copies or substantial portions of the Software.
 *
 *	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *	THE SOFTWARE.
 */
#ifndef MUTEX_GUARD_H
#define MUTEX_GUARD_H
#include "ros/ros.h"
#include "remote_mutex/mutexSrv.h"

/**
 * This class impements a remote mutex guard.
 */
class MutexGuard {
	private:
		/**
		 * The ROS node handle
		 */
		ros::NodeHandle _nh;
	
		/**
		 * The service server.
		 */
		 ros::ServiceServer _mutexServer;
		 
		 /**
		  * The mutex name.
		  */
		  std::string _name;
		  
		  /**
		   * The current mutex holder.
		   */
		  std::string holderName;
		  
		  /**
		   * The guarded variable.
		   */
		   bool locked;
		   
		   /**
		    * Try to lock the mutex.
		    * @return true if we succeeded in locking the mutex
		    */
		   bool tryLock(std::string holder);
		   
		   /**
		    * The ROS Service callback for the mutex.
		    */
		   bool serviceCallback(remote_mutex::mutexSrv::Request &, remote_mutex::mutexSrv::Response &);
		   
		   /**
		    * Try to unlock the mutex for the given holder.
		    * @param holder the holder.
		    * @return true if the mutex has been unlocked sucessfully
		    */
		   bool unlock(std::string holder);
		   
	public:
		
		/**
		 * The Constructor.
		 * @param mutexName
		 */
		MutexGuard(std::string mutexName);
		
		/**
		 * Gets the mutex status.
		 * @return true if the mutex is locked.
		 */
		bool getStatus();
		
		/**
		 * Returns the current mutex holder.
		 * @return the name of the holder, or an empty string if noone is holding the mutex.
		 */
		std::string getHolder();
	
};
#endif
