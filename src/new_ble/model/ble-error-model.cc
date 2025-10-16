/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 The Boeing Company
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
 * Author: Gary Pei <guangyu.pei@boeing.com>
 */
#include "ble-error-model.h"
#include <ns3/log.h>

#define _USE_MATH_DEFINES
#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("BleErrorModel");
NS_OBJECT_ENSURE_REGISTERED (BleErrorModel);

TypeId
BleErrorModel::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::BleErrorModel")
		.SetParent<Object> ()
		.AddConstructor<BleErrorModel> ()
	;
	return tid;
}

BleErrorModel::BleErrorModel (void)
{

}

long double 
BleErrorModel::GetBER (double snr) const
{
  long double z = sqrtl((long double)snr);
	if (snr > 0)
	{
  	return erfcl(z)/2;
	}
	else
	{
		return 0;
	}
}

} // namespace ns3
