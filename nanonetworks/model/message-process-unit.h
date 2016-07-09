/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013,2014 TELEMATICS LAB, DEI - Politecnico di Bari
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
 * Author: Giuseppe Piro <peppe@giuseppepiro.com>, <g.piro@poliba.it>
 */

#ifndef MESSAGE_PROCESS_UNIT_H
#define MESSAGE_PROCESS_UNIT_H

#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/callback.h"
#include "ns3/traced-callback.h"

namespace ns3 {

class SimpleNanoDevice;

/**
 * \ingroup nanonetworks
 *
 * This class provides
 */
class MessageProcessUnit : public Object
{
public:
  static TypeId GetTypeId (void);

  MessageProcessUnit (void);
  virtual ~MessageProcessUnit (void);

  virtual void DoDispose (void);

  void SetDevice (Ptr<SimpleNanoDevice> d);
  Ptr<SimpleNanoDevice> GetDevice (void);

  void CreteMessage ();
  void CreteMessagePlusOne ();
  void CreteMessageHeader36 ();
  void CreteMessageHeader43 ();
  void CreteMessageHeader50 ();
  void ProcessMessage (Ptr<Packet> p);

  void SetPacketSize (int s);
  void SetInterarrivalTime (double t);
  static int RxCount;
  static int RpktCount;
  static int BufferSize;
  static double BW_backhaul;//backhaul bandwidth limit 
  static double ratio_backhaul;//ratio of BW allocated to M2M
  static double t_burst;//burst duration of each poll
  static double T_poll;
  static int RandToken;
  int PktCount;

private:

  Ptr<SimpleNanoDevice> m_device;
  int m_packetSize;
  double m_interarrivalTime;
  TracedCallback<int, int,int,int> m_outTX;
  TracedCallback<int, int, int, int, double, double> m_outRX;
};


} // namespace ns3

#endif /* MESSAGE_PROCESS_UNIT_H */
