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


#ifndef FLOODING_NANO_ROUTING_ENTITY_H
#define FLOODING_NANO_ROUTING_ENTITY_H

#include "nano-routing-entity.h"
#include <list>

namespace ns3 {

class Packet;
class SimpleNanoDevice;
class NanoMacQueue;

/**
 * \ingroup nanonetworks
 *
 * This class implements the Flooding Routing protocol, as described in
 *
 * Giuseppe Piro, Luigi Alredo Grieco, Gennaro Boggia, and Pietro Camarda,"
 * Nano-Sim: simulating electromagnetic-based nanonetworks in the Network Simulator 3",
 * in Proc. of Workshop on NS- 3 (held in conjunction with SIMUTools 2013),
 * Cannes, France, Mar., 2013
 *
 */
class FloodingNanoRoutingEntity : public NanoRoutingEntity
{
public:
  static TypeId GetTypeId (void);

  FloodingNanoRoutingEntity (void);
  virtual ~FloodingNanoRoutingEntity (void);

  virtual void DoDispose (void);

  virtual void SendPacket (Ptr<Packet> p);
  virtual void ReceivePacket (Ptr<Packet> p);
  virtual void ForwardPacket (Ptr<Packet> p);

//Selective flooding implementation
public:
  void UpdateReceivedPacketId (uint32_t id);
  bool CheckAmongReceivedPacket (uint32_t id);
  void SetReceivedPacketListDim (int m);
  void L3CreteMessage ();
  void L3CreteMessagePlusOne ();
  void L3CreteMessageHeader36 ();
  int TTL = 100;
  int KfTTL = 0;
  int RxCount = 0;
  int SinkRx36 = 0;
  int RandValue;
  static int PartialRxCount; 

//header dup recording
  std::pair<int,int> DupCountCurrent;//use 99999 to represent inf
  std::pair<int,int> DupCountPrev;
  int TTLtag = 0; // TTLtag=1:bad channel, choose TTLmin;TTLtag=0:good channel,choose TTLmax
  int DupCount = 0;
private:
  std::list<uint32_t> m_receivedPacketList;
  int m_receivedPacketListDim;

};


} // namespace ns3

#endif /* FLOODING_NANO_ROUTING_ENTITY_H */
