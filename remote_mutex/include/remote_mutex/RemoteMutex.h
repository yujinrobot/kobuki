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
#include "ros/ros.h"
#include "remote_mutex/mutexSrv.h"
#ifndef REMOTE_MUTEX_H
#define REMOTE_MUTEX_H
/**
 * Impements a remote mutex. This is the client.
 */
class RemoteMutex {
	private:
		/**
		 * The ROS Node Handle.
		 */
		ros::NodeHandle _nh;
		
		/**
		 * The Client to the mutex serivce.
		 */
		 ros::ServiceClient _client;
		 
		 /**
		  * The Remote Mutex Name.
		  */
		 std::string _remoteMutexName;
		
	public:
		/**
		 * Constructor.
		 * @param name the name of the mutex.
		 */
		RemoteMutex(std::string name);
		
		/**
		 * Try to lock once the mutex. 
		 * @return true if the mutex was successfuly locked
		 */
		bool tryLock();
		
		/**
		 * Retrive the current status of the mutex.
		 */
		bool getStatus();
		
		/**
		 * Will try to lock the mutex for a give time. 
		 * @param timeout the timeout wait for locking.
		 * @return true if we have sucessfully locked the mutex
		 */
		bool lock(ros::Duration timeout = ros::Duration(0));
		
		/**
		 * Will unlock the mutex.
		 * @return true if the mutex was unlocked sucessfully. False otherwise.
		 */
		bool unlock();		
};
#endif
