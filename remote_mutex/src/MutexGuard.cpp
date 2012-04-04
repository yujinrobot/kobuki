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
#include "remote_mutex/MutexGuard.h"

MutexGuard::MutexGuard(std::string mutexName) {
	holderName = "";
	locked = false;
	_name = "/mutexes/" + mutexName;
	_mutexServer = _nh.advertiseService(_name,&MutexGuard::serviceCallback,this);
}

bool MutexGuard::tryLock(std::string holder) {
	if (locked) {
		 ROS_INFO("[MutexGuard] %s tried to lock %s but failed.",holder.c_str(),_name.c_str());
		 return false;
	 }
	locked = true;
	holderName = holder;
	ROS_INFO("[MutexGuard] %s sucessfully locked mutex %s.",holder.c_str(),_name.c_str());
	return true;
}

bool MutexGuard::unlock(std::string holder) {
	if (!locked) {
		 ROS_ERROR("[MutexGuard] Node %s tried to unlock already unlocked mutex %s",holder.c_str(),_name.c_str());
		 return false;		 
	 }
	if (holder != holderName) {
		 ROS_ERROR("[MutexGuard] %s tried to unlock %s mutex that does not hold",holder.c_str(),_name.c_str());
		 return false;
	 }
	holderName = "";
	locked = false;
	return true;	
}

bool MutexGuard::getStatus() {
	return locked;
}

std::string MutexGuard::getHolder() {
	return holderName;
}

bool MutexGuard::serviceCallback(remote_mutex::mutexSrv::Request& rq, remote_mutex::mutexSrv::Response& rs) {
	switch (rq.requestType) {
		case remote_mutex::mutexSrv::Request::TYPE_LOCK :
			if (tryLock(rq.requestor)) {
				rs.status = remote_mutex::mutexSrv::Response::STATUS_LOCKED;
			} else {
				rs.status = remote_mutex::mutexSrv::Response::STATUS_UNLOCKED;
			}
			break;
		case remote_mutex::mutexSrv::Request::TYPE_UNLOCK :
			if (unlock(rq.requestor)) {
				rs.status = remote_mutex::mutexSrv::Response::STATUS_UNLOCKED;
			} else {
				rs.status = remote_mutex::mutexSrv::Response::STATUS_LOCKED;
			}
			break;
		case remote_mutex::mutexSrv::Request::TYPE_POLL :	
			if (getStatus()) {
				rs.status = remote_mutex::mutexSrv::Response::STATUS_LOCKED;
			} else {
				rs.status = remote_mutex::mutexSrv::Response::STATUS_UNLOCKED;
			}		
			break;
		default:
			return false;
	}
	
	return true;
}


