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
#include "remote_mutex/RemoteMutex.h"

RemoteMutex::RemoteMutex(std::string name) {
	_remoteMutexName = "/mutexes/"+name;
	
	int retries = 0;
	while (!ros::service::waitForService(_remoteMutexName, ros::Duration(.1)) && ros::ok()) {
		retries++;
		if (retries > 10) 
			ROS_ERROR("[RemoteMutex] Could not find %s",_remoteMutexName.c_str());
		ros::spinOnce();
	}
	
	_client = _nh.serviceClient<remote_mutex::mutexSrv>(_remoteMutexName);
	
}

bool RemoteMutex::getStatus() {
	remote_mutex::mutexSrv srv;
	
	srv.request.requestor =  ros::this_node::getName();
	srv.request.requestType = remote_mutex::mutexSrv::Request::TYPE_POLL;
	if (_client.call(srv)) {
		return srv.response.status == remote_mutex::mutexSrv::Response::STATUS_LOCKED;
	} else {
		ROS_ERROR("Failed to call mutex %s service",_remoteMutexName.c_str());
		return false;
	}
}

bool RemoteMutex::tryLock() {
	remote_mutex::mutexSrv srv;
	
	srv.request.requestor =  ros::this_node::getName();
	srv.request.requestType = remote_mutex::mutexSrv::Request::TYPE_LOCK;
	if (_client.call(srv)) {
		return srv.response.status == remote_mutex::mutexSrv::Response::STATUS_LOCKED;
	} else {
		ROS_ERROR("Failed to call mutex %s service",_remoteMutexName.c_str());
		return false;
	}
}

bool RemoteMutex::unlock() {
	remote_mutex::mutexSrv srv;
	
	srv.request.requestor =  ros::this_node::getName();
	srv.request.requestType = remote_mutex::mutexSrv::Request::TYPE_UNLOCK;
	if (_client.call(srv)) {
		return srv.response.status == remote_mutex::mutexSrv::Response::STATUS_UNLOCKED;
	} else {
		ROS_ERROR("Failed to call mutex %s service",_remoteMutexName.c_str());
		return false;
	}
}

bool RemoteMutex::lock(ros::Duration timeout) {
	ros::Time start = ros::Time::now();
	
	while(ros::Time::now() < start + timeout || timeout == ros::Duration(0)) {		
		if (tryLock()) {
			ROS_INFO("Locked sucessfully");
			return true;
		}
		ros::spinOnce();
		if (!ros::ok()) return false;
	}
	
	return false;
}
