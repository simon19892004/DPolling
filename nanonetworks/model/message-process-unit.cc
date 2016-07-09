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


#include "message-process-unit.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/packet.h"
#include "simple-nano-device.h"
#include "nano-mac-queue.h"
#include "nano-mac-entity.h"
#include "nano-routing-entity.h"
#include "flooding-nano-routing-entity.h"
#include "nano-spectrum-phy.h"
#include "nano-mac-header.h"
#include "ns3/seq-ts-header.h"
#include "ns3/simulator.h"
#include "ns3/nano-l3-header.h"
#include <math.h>
#include "ns3/core-module.h"

NS_LOG_COMPONENT_DEFINE ("MessageProcessUnit");


namespace ns3 {


NS_OBJECT_ENSURE_REGISTERED (MessageProcessUnit);

//UniformVariable random;

int MessageProcessUnit::RxCount = 0;
int MessageProcessUnit::RpktCount = 1;
int MessageProcessUnit::BufferSize = 10;//packets
int MessageProcessUnit::RandToken = 0;
double MessageProcessUnit::BW_backhaul = 20000000;//backhaul bandwidth limit 10mbps
double MessageProcessUnit::ratio_backhaul = 1.0; //= random.GetValue(0.0,1.0);//ratio of BW allocated to M2M
double MessageProcessUnit::t_burst = 0.001;//burst duration of each poll (s)
double MessageProcessUnit::T_poll = 0.0;

TypeId MessageProcessUnit::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MessageProcessUnit")
    .SetParent<Object> ()
    .AddTraceSource ("outTX",  "outTX",  MakeTraceSourceAccessor (&MessageProcessUnit::m_outTX))
    .AddTraceSource ("outRX",  "outRX",  MakeTraceSourceAccessor (&MessageProcessUnit::m_outRX));
;
  return tid;
}


MessageProcessUnit::MessageProcessUnit ()
{
  //NS_LOG_FUNCTION (this);
  m_device = 0;
  m_packetSize = 0;
  m_interarrivalTime = 99999999999;
}


MessageProcessUnit::~MessageProcessUnit ()
{
  //NS_LOG_FUNCTION (this);
}

void 
MessageProcessUnit::DoDispose (void)
{
  //NS_LOG_FUNCTION (this);
  m_device = 0;
}

void
MessageProcessUnit::SetDevice (Ptr<SimpleNanoDevice> d)
{
  //NS_LOG_FUNCTION (this);
  m_device = d;
}

Ptr<SimpleNanoDevice>
MessageProcessUnit::GetDevice (void)
{
  return m_device;
}

void
MessageProcessUnit::CreteMessage ()
{
  
  
  
  uint8_t *buffer  = new uint8_t[m_packetSize=100*BufferSize];
  for (int i = 0; i < m_packetSize; i++)
    {
	  buffer[i] = 129;
    }
  Ptr<Packet> p = Create<Packet>(buffer, m_packetSize);
  //NS_LOG_FUNCTION("pkt_without header! NodeID "<<GetDevice ()->GetNode ()->GetId ()<<"Time: "<<Simulator::Now ().GetSeconds ()<<"pkt-id "<<p->GetUid ()<<"pkt-size:"<<p->GetSize());
  //NS_LOG_FUNCTION("pkt-size no header: "<<p->GetSize());
  SeqTsHeader seqTs;
  seqTs.SetSeq (p->GetUid ());
  p->AddHeader (seqTs);
  //NS_LOG_FUNCTION("pkt-size with header: "<<p->GetSize());
  //m_outTX ((int)p->GetUid (), (int)GetDevice ()->GetNode ()->GetId (),(int)GetDevice ()->GetObject <SimpleNanoDevice> ()->m_type);
  m_outTX ((int)p->GetUid (), (int)GetDevice ()->GetNode ()->GetId (),(int)GetDevice ()->m_type, (int)p->GetSize());
  ////NS_LOG_FUNCTION(this<<"generateMessage at:"<<seqTs.GetTs ()/1e15);
  //NS_LOG_FUNCTION("Send to L3! NodeID "<<GetDevice ()->GetNode ()->GetId ()<<"Time: "<<Simulator::Now ().GetSeconds ()<<"pkt-id "<<p->GetUid ()<<"pkt-size:"<<p->GetSize()); 
    NS_LOG_FUNCTION(Simulator::Now ().GetSeconds ()<<" Send to L3! NodeID "<<GetDevice ()->GetNode ()->GetId ()<<"pkt-size:"<<p->GetSize()<<" pkt-id "<<p->GetUid ());
  //****************************check neighbour before go to L3*******************************//
  Ptr<NanoMacEntity> mac = GetDevice ()->GetMac ();
  mac -> CheckForNeighbors();
  //******************************************************************************************//
  m_device->SendPacket (p);
  //if (GetDevice ()->GetNode ()->GetId () == 5)
  
  //Simulator::Schedule (Seconds (m_interarrivalTime), &MessageProcessUnit::CreteMessage, this);//tune this one to switch between event-driven and polling
}

void
MessageProcessUnit::CreteMessagePlusOne ()
{
  
  
  
  uint8_t *buffer  = new uint8_t[m_packetSize=100*(BufferSize+1)];
  for (int i = 0; i < m_packetSize; i++)
    {
	  buffer[i] = 129;
    }
  Ptr<Packet> p = Create<Packet>(buffer, m_packetSize);
  //NS_LOG_FUNCTION("pkt_without header! NodeID "<<GetDevice ()->GetNode ()->GetId ()<<"Time: "<<Simulator::Now ().GetSeconds ()<<"pkt-id "<<p->GetUid ()<<"pkt-size:"<<p->GetSize());
  //NS_LOG_FUNCTION("pkt-size no header: "<<p->GetSize());
  SeqTsHeader seqTs;
  seqTs.SetSeq (p->GetUid ());
  p->AddHeader (seqTs);
  //NS_LOG_FUNCTION("pkt-size with header: "<<p->GetSize());
  //m_outTX ((int)p->GetUid (), (int)GetDevice ()->GetNode ()->GetId (),(int)GetDevice ()->GetObject <SimpleNanoDevice> ()->m_type);
  m_outTX ((int)p->GetUid (), (int)GetDevice ()->GetNode ()->GetId (),(int)GetDevice ()->m_type, (int)p->GetSize());
  ////NS_LOG_FUNCTION(this<<"generateMessage at:"<<seqTs.GetTs ()/1e15);
  //NS_LOG_FUNCTION("Send to L3! NodeID "<<GetDevice ()->GetNode ()->GetId ()<<"Time: "<<Simulator::Now ().GetSeconds ()<<"pkt-id "<<p->GetUid ()<<"pkt-size:"<<p->GetSize()); 
    NS_LOG_FUNCTION(Simulator::Now ().GetSeconds ()<<" Send to L3! NodeID "<<GetDevice ()->GetNode ()->GetId ()<<"pkt-size:"<<p->GetSize()<<" pkt-id "<<p->GetUid ());
  //****************************check neighbour before go to L3*******************************//
  Ptr<NanoMacEntity> mac = GetDevice ()->GetMac ();
  mac -> CheckForNeighbors();
  //******************************************************************************************//
  m_device->SendPacket (p);
  //if (GetDevice ()->GetNode ()->GetId () == 5)
  
  //Simulator::Schedule (Seconds (m_interarrivalTime), &MessageProcessUnit::CreteMessage, this);//tune this one to switch between event-driven and polling
}

void
MessageProcessUnit::CreteMessageHeader36 ()
{
  
  
  
  uint8_t *buffer  = new uint8_t[m_packetSize=100];
  for (int i = 0; i < m_packetSize; i++)
    {
	  buffer[i] = 129;
    }
  Ptr<Packet> p = Create<Packet>(buffer, m_packetSize);
  //NS_LOG_FUNCTION("pkt_without header! NodeID "<<GetDevice ()->GetNode ()->GetId ()<<"Time: "<<Simulator::Now ().GetSeconds ()<<"pkt-id "<<p->GetUid ()<<"pkt-size:"<<p->GetSize());
  //NS_LOG_FUNCTION("pkt-size no header: "<<p->GetSize());
  SeqTsHeader seqTs;
  seqTs.SetSeq (p->GetUid ());
  p->AddHeader (seqTs);
  p->RemoveAtEnd(100);
  //NS_LOG_FUNCTION("pkt-size with header: "<<p->GetSize());
  //m_outTX ((int)p->GetUid (), (int)GetDevice ()->GetNode ()->GetId (),(int)GetDevice ()->GetObject <SimpleNanoDevice> ()->m_type);
  m_outTX ((int)p->GetUid (), (int)GetDevice ()->GetNode ()->GetId (),(int)GetDevice ()->m_type,(int)p->GetSize());
  ////NS_LOG_FUNCTION(this<<"generateMessage at:"<<seqTs.GetTs ()/1e15);
  //NS_LOG_FUNCTION("Send to L3! NodeID "<<GetDevice ()->GetNode ()->GetId ()<<"Time: "<<Simulator::Now ().GetSeconds ()<<"pkt-id "<<p->GetUid ()<<"pkt-size:"<<p->GetSize());
    NS_LOG_FUNCTION(Simulator::Now ().GetSeconds ()<<" Send to L3! NodeID "<<GetDevice ()->GetNode ()->GetId ()<<"pkt-size:"<<p->GetSize()<<" pkt-id "<<p->GetUid ());
  //****************************check neighbour before go to L3*******************************//
  Ptr<NanoMacEntity> mac = GetDevice ()->GetMac ();
  mac -> CheckForNeighbors();
  //******************************************************************************************//
  m_device->SendPacket (p);
  if (GetDevice ()->m_type == SimpleNanoDevice::NanoInterface)
  {
         
         T_poll = Simulator::Now ().GetNanoSeconds ();
        NS_LOG_FUNCTION("beacon "<<T_poll<<" NanoSeconds");
         Simulator::Schedule (Seconds (1e-2), &MessageProcessUnit::CreteMessageHeader43, this);//tune this one to switch between event-driven and polling   
         Simulator::Schedule (Seconds (m_interarrivalTime), &MessageProcessUnit::CreteMessageHeader36, this);//tune this one to switch between event-driven and polling
  }
  
}

void
MessageProcessUnit::CreteMessageHeader43 ()//sinks'operation
{
  
  
  
  uint8_t *buffer  = new uint8_t[m_packetSize=100];
  for (int i = 0; i < m_packetSize; i++)
    {
	  buffer[i] = 129;
    }
  Ptr<Packet> p = Create<Packet>(buffer, m_packetSize);
  //NS_LOG_FUNCTION("pkt_without header! NodeID "<<GetDevice ()->GetNode ()->GetId ()<<"Time: "<<Simulator::Now ().GetSeconds ()<<"pkt-id "<<p->GetUid ()<<"pkt-size:"<<p->GetSize());
  //NS_LOG_FUNCTION("pkt-size no header: "<<p->GetSize());
  SeqTsHeader seqTs;
  seqTs.SetSeq (p->GetUid ());
  p->AddHeader (seqTs);
  p->RemoveAtEnd(93);
  //NS_LOG_FUNCTION("pkt-size with header: "<<p->GetSize());
  //m_outTX ((int)p->GetUid (), (int)GetDevice ()->GetNode ()->GetId (),(int)GetDevice ()->GetObject <SimpleNanoDevice> ()->m_type);
  ////NS_LOG_FUNCTION(this<<"generateMessage at:"<<seqTs.GetTs ()/1e15);
  //NS_LOG_FUNCTION("Send to L3! NodeID "<<GetDevice ()->GetNode ()->GetId ()<<"Time: "<<Simulator::Now ().GetSeconds ()<<"pkt-id "<<p->GetUid ()<<"pkt-size:"<<p->GetSize());
    NS_LOG_FUNCTION(Simulator::Now ().GetSeconds ()<<" Send to L3! NodeID "<<GetDevice ()->GetNode ()->GetId ()<<"pkt-size:"<<p->GetSize()<<" pkt-id "<<p->GetUid ());
  //****************************check neighbour before go to L3*******************************//
  Ptr<NanoMacEntity> mac = GetDevice ()->GetMac ();
  mac -> CheckForNeighbors();
  //**********************************Buffer control*********************************************//
  RpktCount = 1; 
  //MessageProcessUnit::ratio_backhaul = random.GetValue(0.0,1.0);
  NS_LOG_FUNCTION("Rx: "<<RxCount);
  if (RxCount != 0 && ratio_backhaul*BW_backhaul*t_burst >= 100)
  {
                
        
        BufferSize = floor(ratio_backhaul*BW_backhaul*t_burst/RxCount/100);
        if (BufferSize > 10)
        {
                BufferSize = 10;
        }
        NS_LOG_FUNCTION("BK_Ratio "<<ratio_backhaul<<"Buffer: "<<BufferSize);
        
        

        if (BufferSize > 0 && (ratio_backhaul*BW_backhaul*t_burst - RxCount*100*BufferSize == 0 || ratio_backhaul*BW_backhaul*t_burst - RxCount*100*BufferSize < 100) || BufferSize == 10)//if (BufferSize != 0)//no redundant bandwidth
        {
                m_outTX ((int)p->GetUid (), (int)GetDevice ()->GetNode ()->GetId (),(int)GetDevice ()->m_type,(int)p->GetSize());                
                m_device->SendPacket (p);
        }

        if (BufferSize ==0 || ratio_backhaul*BW_backhaul*t_burst - RxCount*100*BufferSize >= 100 && BufferSize != 10)//if (BufferSize == 0)//redundant bandwidth
        {
                RandToken = 0;                
                for (int i=RxCount-1;i>=1;i--)
                {
                       if (floor((ratio_backhaul*BW_backhaul*t_burst - RxCount*100*BufferSize)/i/100) >= 1)//if (floor(ratio_backhaul*BW_backhaul*t_burst/i/100) >= 1)
                        {
                                RandToken = floor(float(i)/float(RxCount)*100);
                                //BufferSize = 1;
                                NS_LOG_FUNCTION("pktPlusOne nodes to be polled "<<i<<" "<<RandToken<<" % ");
                                
                                break;
                        }
                        else
                        {
                                continue;
                        } 
                        
                }
        
                        if (GetDevice ()->m_type == SimpleNanoDevice::NanoInterface && RandToken > 0)//if (GetDevice ()->m_type == SimpleNanoDevice::NanoInterface && BufferSize == 1)
                        {
         
                                Simulator::Schedule (Seconds (0), &MessageProcessUnit::CreteMessageHeader50, this);//tune this one to switch between event-driven and polling   
                        }
        }

               
        RxCount = 0;
  }
  else
{
        RxCount = 0;        
        NS_LOG_FUNCTION("Not enough bk! Rx Refresh: "<<RxCount);
}
  
  
  
  //Simulator::Schedule (Seconds (m_interarrivalTime), &MessageProcessUnit::CreteMessageHeader43, this);//tune this one to switch between event-driven and polling
}

void
MessageProcessUnit::CreteMessageHeader50 ()//sinks'operation
{
  
  
  
  uint8_t *buffer  = new uint8_t[m_packetSize=100];
  for (int i = 0; i < m_packetSize; i++)
    {
	  buffer[i] = 129;
    }
  Ptr<Packet> p = Create<Packet>(buffer, m_packetSize);
  //NS_LOG_FUNCTION("pkt_without header! NodeID "<<GetDevice ()->GetNode ()->GetId ()<<"Time: "<<Simulator::Now ().GetSeconds ()<<"pkt-id "<<p->GetUid ()<<"pkt-size:"<<p->GetSize());
  //NS_LOG_FUNCTION("pkt-size no header: "<<p->GetSize());
  SeqTsHeader seqTs;
  seqTs.SetSeq (p->GetUid ());
  p->AddHeader (seqTs);
  p->RemoveAtEnd(86);
  //NS_LOG_FUNCTION("pkt-size with header: "<<p->GetSize());
  //m_outTX ((int)p->GetUid (), (int)GetDevice ()->GetNode ()->GetId (),(int)GetDevice ()->GetObject <SimpleNanoDevice> ()->m_type);
  m_outTX ((int)p->GetUid (), (int)GetDevice ()->GetNode ()->GetId (),(int)GetDevice ()->m_type,(int)p->GetSize());
  ////NS_LOG_FUNCTION(this<<"generateMessage at:"<<seqTs.GetTs ()/1e15);
  //NS_LOG_FUNCTION("Send to L3! NodeID "<<GetDevice ()->GetNode ()->GetId ()<<"Time: "<<Simulator::Now ().GetSeconds ()<<"pkt-id "<<p->GetUid ()<<"pkt-size:"<<p->GetSize());
    NS_LOG_FUNCTION(Simulator::Now ().GetSeconds ()<<"Send to L3! NodeID "<<GetDevice ()->GetNode ()->GetId ()<<"pkt-size:"<<p->GetSize()<<" pkt-id "<<p->GetUid ());
  //****************************check neighbour before go to L3*******************************//
  Ptr<NanoMacEntity> mac = GetDevice ()->GetMac ();
  mac -> CheckForNeighbors();

  m_device->SendPacket (p);

  
  
  
  //Simulator::Schedule (Seconds (m_interarrivalTime), &MessageProcessUnit::CreteMessageHeader43, this);//tune this one to switch between event-driven and polling
}

void
MessageProcessUnit::ProcessMessage (Ptr<Packet> p)
{
  //NS_LOG_FUNCTION (this);
  
  
  NanoL3Header l3Header;
  p->RemoveHeader (l3Header);
  //NS_LOG_FUNCTION("ttl"<<l3Header.GetTtl());
  
  SeqTsHeader seqTs;
  p->RemoveHeader (seqTs);

  //NS_LOG_FUNCTION (this << l3Header);
  //NS_LOG_FUNCTION (this << seqTs);

  double delay =  Simulator::Now ().GetPicoSeconds () - seqTs.GetTs ().GetPicoSeconds ();
  //double delay =  Simulator::Now ().GetNanoSeconds () - T_poll;
  
  
 
  if (p->GetSize() >= 100)
  {  
        NS_LOG_FUNCTION("Pkt Arrival "<<Simulator::Now ().GetNanoSeconds ()<<" Poll "<<T_poll<<" delay "<<delay);
        RpktCount++;
        Ptr<NanoRoutingEntity> L3 = GetDevice ()->GetObject <SimpleNanoDevice>()->GetL3 ();
        Ptr<FloodingNanoRoutingEntity> L3Flood = L3->GetObject<FloodingNanoRoutingEntity>();
        int T = L3Flood->TTL;         
        m_outRX (seqTs.GetSeq (), p->GetSize (), (int)l3Header.GetSource (), (int)GetDevice ()->GetNode ()->GetId (), delay, ratio_backhaul);
  }
}


void
MessageProcessUnit::SetPacketSize (int s)
{
  //NS_LOG_FUNCTION (this);
  m_packetSize = s;
}

void
MessageProcessUnit::SetInterarrivalTime (double t)
{
  //NS_LOG_FUNCTION (this);
  m_interarrivalTime = t;
}

} // namespace ns3
