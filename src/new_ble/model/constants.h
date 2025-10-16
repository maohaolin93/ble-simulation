/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 KULeuven 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Stijn Geysen <stijn.geysen@student.kuleuven.be>
 */


#ifndef BLE_CONSTANTS_H
#define BLE_CONSTANTS_H

#include <ns3/nstime.h>

#define BLE_CONST_TIME_UNIT Time::Unit::US
#define QUEUE_SIZE_PACKETS "100p" // Max number of packets in the queue
#define T_IFS 150 // microseconds
#define PRECISION 100 // In NanoSeconds

#endif // BLE_CONSTANTS_H
