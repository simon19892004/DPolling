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


#include "flooding-nano-routing-entity.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/packet.h"
#include "simple-nano-device.h"
#include "nano-mac-queue.h"
#include "nano-l3-header.h"
#include "nano-mac-entity.h"
#include "ns3/log.h"
#include "ns3/queue.h"
#include "ns3/simulator.h"
#include "ns3/enum.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/pointer.h"
#include "ns3/channel.h"
#include "simple-nano-device.h"
#include "nano-spectrum-phy.h"
#include "nano-mac-entity.h"
#include "nano-mac-header.h"
#include "ns3/seq-ts-header.h"
#include "ns3/simulator.h"
#include "nano-routing-entity.h"
#include "message-process-unit.h"
#include "ns3/core-module.h"
#include <math.h>
#include <stdlib.h>

NS_LOG_COMPONENT_DEFINE ("FloodingNanoRoutingEntity");

namespace ns3 {


NS_OBJECT_ENSURE_REGISTERED (FloodingNanoRoutingEntity);
int FloodingNanoRoutingEntity::PartialRxCount = 0;
UniformVariable random;

TypeId FloodingNanoRoutingEntity::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::FloodingNanoRoutingEntity")
    .SetParent<Object> ();
  return tid;
}


FloodingNanoRoutingEntity::FloodingNanoRoutingEntity ()
{
  SetDevice(0);
  m_receivedPacketListDim = 300;
  DupCountCurrent.first = 0;
  DupCountCurrent.second = 9999;
  DupCountPrev.first = 0;
  DupCountPrev.second = 9999;
  for (int i = 0; i < m_receivedPacketListDim; i++)
    {
	  m_receivedPacketList.push_back (9999999);
    }
   
}


FloodingNanoRoutingEntity::~FloodingNanoRoutingEntity ()
{
  SetDevice(0);
}

void 
FloodingNanoRoutingEntity::DoDispose (void)
{
  SetDevice (0);
}

void
FloodingNanoRoutingEntity::SendPacket (Ptr<Packet> p)
{
  //NS_LOG_FUNCTION (this << p << "size" << p->GetSize ());

  SeqTsHeader seqTs;
  p->RemoveHeader (seqTs);

  //NS_LOG_FUNCTION (this << p << "size" << p->GetSize () << seqTs);

  NanoL3Header header;
  uint32_t src = GetDevice ()->GetNode ()->GetId ();
  uint32_t dst = 0;
  uint32_t id = seqTs.GetSeq ();
  uint32_t ttl = TTL;
  header.SetSource (src);
  header.SetDestination (dst);
  header.SetTtl (ttl);
  header.SetPacketId (id);
  //NS_LOG_FUNCTION (this << "l3 header" << header);

  p->AddHeader (seqTs);
  p->AddHeader (header);
  //NS_LOG_FUNCTION ("Id "<< src << "pkt size " << p->GetSize ());

  UpdateReceivedPacketId (id);

  SenderTypeTag tag;
  
  //***********************************************************//
  if (GetDevice ()->m_type == SimpleNanoDevice::NanoInterface)
    {  
        if (p->GetSize () == 28)//control beacon
        {
                tag.type = 4;
        }
        else if (p->GetSize () == 35)
        {
                tag.type = 0;//polling beacon
        }
        else
        {
                tag.type = 5;//partial polling beacon
        }        
        
    }
  //***********************************************************//
  else if (GetDevice ()->m_type == SimpleNanoDevice::NanoNode)
    {
	  tag.type = 1;
    }
  else if(GetDevice ()->m_type == SimpleNanoDevice::NanoRouter)
    {
	  tag.type = 2;
    }
  else
    {
      
    }
    
  if (tag.type == 0 || header.GetTtl() != 100)
  {
    //NS_LOG_FUNCTION ("L3Send: NodeID"<< GetDevice()->GetNode()->GetId() <<"pkt-id "<< p->GetUid() <<"pkt-size " << p->GetSize ()<<"ttl: "<<header.GetTtl());
  }

  //NS_LOG_FUNCTION ("L3Send: NodeID"<< GetDevice()->GetNode()->GetId() <<"pkt-id "<< p->GetUid() <<"pkt-size " << p->GetSize ()<<"ttl: "<<header.GetTtl()<<"tag: "<<tag.type);

  p->AddPacketTag (tag);
  
  
  
  Ptr<NanoMacEntity> mac = GetDevice ()->GetMac ();
  mac->Send (p, dst);
}

void
FloodingNanoRoutingEntity::ReceivePacket (Ptr<Packet> pkt)
{
  //NS_LOG_FUNCTION (this << pkt << "size" << pkt->GetSize());
  
  Ptr<Packet> p = pkt->Copy ();
  
  NanoMacHeader macHeader;
  p->RemoveHeader (macHeader);
  SenderTypeTag tag;
  p->RemovePacketTag (tag);
  NanoL3Header l3Header;
  p->RemoveHeader (l3Header);
  
  if (tag.type == 0)
  {
    //NS_LOG_FUNCTION ("L3Receive: NodeID"<< GetDevice()->GetNode()->GetId() <<"pkt-id "<< p->GetUid() <<"pkt-size " << p->GetSize ()<<"ttl: "<<l3Header.GetTtl());
  }
  //NS_LOG_FUNCTION (this << macHeader);
  //NS_LOG_FUNCTION (this << l3Header);
 

  uint32_t id = l3Header.GetPacketId ();
  bool alreadyReceived = CheckAmongReceivedPacket (id);
  UpdateReceivedPacketId (id);

  SimpleNanoDevice::NodeType type = GetDevice ()->m_type;
  
  
   if (!alreadyReceived)
    {
	  //NS_LOG_FUNCTION (this << "received a packet for the first time");
	  KfTTL = KfTTL + l3Header.GetTtl();
	  //RxCount++;
      
      
	  if (GetDevice ()->GetMessageProcessUnit () && type == SimpleNanoDevice::NanoInterface)
	    {
	      //NS_LOG_FUNCTION (this << GetDevice()->GetNode ()->GetId () << l3Header.GetSource () << l3Header.GetDestination () << "FOR ME");             
	      p->AddHeader (l3Header);
              //NS_LOG_FUNCTION ("FOR SINK,size "<<p->GetSize ());
              //NS_LOG_FUNCTION(Simulator::Now ().GetSeconds ()<<" suspended at sink."<<" pkt-id "<<p->GetUid ()<<" tag "<<tag.type<<" pkt-size " << p->GetSize ());
              if (tag.type == 1 && p->GetSize () == 28)
              {
                
                //GetDevice ()->GetMessageProcessUnit ()->ProcessMessage (p);
                GetDevice ()->GetMessageProcessUnit ()-> RxCount++;
                NS_LOG_FUNCTION(Simulator::Now ().GetSeconds ()<<" RxCount: "<<GetDevice ()->GetMessageProcessUnit ()-> RxCount<<" pkt-id "<<p->GetUid ()<<" tag "<<tag.type);
              }
              else if (tag.type == 1 && p->GetSize () >= 100)
                {
                        NS_LOG_FUNCTION(Simulator::Now ().GetSeconds ()<<" RpktCount: "<<GetDevice ()->GetMessageProcessUnit ()-> RpktCount<<" pkt-id "<<p->GetUid ()<<" tag "<<tag.type); 
                }
              
                               
                GetDevice ()->GetMessageProcessUnit ()->ProcessMessage (p);
              
  
	    }
	  else
	    {
          //NS_LOG_FUNCTION (this << GetDevice()->GetNode ()->GetId () << l3Header.GetSource () << l3Header.GetDestination () << "NOT FOR ME");
	      p->AddHeader (l3Header);
	      
	   //****************************************************//   
	      if (tag.type == 0 && type != SimpleNanoDevice::NanoInterface) //&& (TTL == 100 || 100 - TTL <= l3Header.GetTtl()))//polling CONTROL not working
	      {
		  
		TTL = 100 - l3Header.GetTtl();
		p->AddPacketTag(tag);
		//NS_LOG_FUNCTION (Simulator::Now ().GetSeconds ()<<"L3Receive to forward: NodeID"<< GetDevice()->GetNode()->GetId() <<"pkt-id "<< p->GetUid() <<"pkt-size " << p->GetSize ()<<"ttl"<<l3Header.GetTtl()<<"Tag: "<<tag.type);
		ForwardPacket (p);
                Simulator::Schedule (FemtoSeconds (424300), &FloodingNanoRoutingEntity::L3CreteMessage, this);//polling pkt duration:43*100+42*10000=424300 fs
		
	      }
              if (tag.type == 4 && type != SimpleNanoDevice::NanoInterface)// && (TTL == 100 || 100 - TTL <= l3Header.GetTtl()))//controlling
              {
                 TTL = 100 - l3Header.GetTtl();
		p->AddPacketTag(tag);
		ForwardPacket (p);
                //NS_LOG_FUNCTION (Simulator::Now ().GetSeconds ()<<"L3Receive to forward: NodeID"<< GetDevice()->GetNode()->GetId() <<"pkt-id "<< p->GetUid() <<"pkt-size " << p->GetSize ()<<"ttl"<<l3Header.GetTtl()<<"Tag: "<<tag.type);
                PartialRxCount = 0;
                Simulator::Schedule (FemtoSeconds (353600), &FloodingNanoRoutingEntity::L3CreteMessageHeader36, this);//beacon duration:36*100+35*10000=353600fs                      
              }
              if (tag.type == 5 && type != SimpleNanoDevice::NanoInterface)// && (TTL == 100 || 100 - TTL <= l3Header.GetTtl()))
              {
                TTL = 100 - l3Header.GetTtl();
		p->AddPacketTag(tag);
		ForwardPacket (p);
                //NS_LOG_FUNCTION (Simulator::Now ().GetSeconds ()<<"L3Receive to forward: NodeID"<< GetDevice()->GetNode()->GetId() <<"pkt-id "<< p->GetUid() <<"pkt-size " << p->GetSize ()<<"ttl"<<l3Header.GetTtl()<<"Tag: "<<tag.type);
                RandValue = round(random.GetValue(0.0,100.0));
                
                if (RandValue < GetDevice ()->GetMessageProcessUnit ()-> RandToken)
                {
                        PartialRxCount++;                        
                        NS_LOG_FUNCTION(Simulator::Now ().GetSeconds ()<<"Partially polled! RandToken "<<GetDevice ()->GetMessageProcessUnit ()-> RandToken<<"My Value: "<<RandValue<<" PartialRxCount: "<<PartialRxCount);                      
                        Simulator::Schedule (FemtoSeconds (495900), &FloodingNanoRoutingEntity::L3CreteMessagePlusOne, this);//polling pkt duration:50*100+49*10000=495900 fs
                }
                else
                {
                        NS_LOG_FUNCTION(Simulator::Now ().GetSeconds ()<<"RandToken Fail "<<GetDevice ()->GetMessageProcessUnit ()-> RandToken<<"My Value: "<<RandValue<<" PartialRxCount: "<<PartialRxCount);
                        if (GetDevice ()->GetMessageProcessUnit ()-> BufferSize != 0)
                        {
                                PartialRxCount++;                        
                                //NS_LOG_FUNCTION(Simulator::Now ().GetSeconds ()<<"Partially polled! RandToken "<<GetDevice ()->GetMessageProcessUnit ()-> RandToken<<"My Value: "<<RandValue<<" PartialRxCount: "<<PartialRxCount);                      
                                Simulator::Schedule (FemtoSeconds (495900), &FloodingNanoRoutingEntity::L3CreteMessage, this);//polling pkt duration:50*100+49*10000=495900 fs
                        }                        
                        
                } 
              }
	    //****************************************************//
	      if (tag.type == 1 && type != SimpleNanoDevice::NanoInterface)
	        {
	    	  //NS_LOG_FUNCTION (this << "received from a sensor --> forward");
                  if (TTL == 100 || TTL < l3Header.GetTtl())
                  {
                        //NS_LOG_FUNCTION ("L3Receive to forward: NodeID"<< GetDevice()->GetNode()->GetId() <<"pkt-id "<< p->GetUid() <<"pkt-size " << p->GetSize ()<<"ttl"<<l3Header.GetTtl()<<"Tag: "<<tag.type<<" NodeTTL "<<TTL);                        
                        p->AddPacketTag(tag);                        
                        ForwardPacket (p);
                  }              
	    	  
	        }
	      if (tag.type == 2 && type == SimpleNanoDevice::NanoRouter)
	    	{
	    	  //NS_LOG_FUNCTION (this << "received from a router, i'm a router --> forward");
	    	  if (TTL == 100 || TTL < l3Header.GetTtl())
                  {
                        ForwardPacket (p);
                  }
	    	}

	    }
    }
  else
    {
	  //NS_LOG_FUNCTION (this << "packet already received in the past");
	  //**********************statistics of Dup_header**************************************************
	  if (tag.type == 0 && GetDevice ()->m_type != SimpleNanoDevice::NanoInterface)
	  {
	      
	      DupCount++;
	      if (p->GetUid() != DupCountCurrent.first && p->GetUid() != DupCountPrev.first)//new pkt dup(pkt 0 is regarded to be old by default)
	      {
		  
		  if (DupCountCurrent.second != 9999 && DupCountPrev.second != 9999)//jump over null round
		  {
		    
		    if (DupCountCurrent.second <= DupCountPrev.second)
		    {
		    
			//TTLtag = 1;//bad channel
                        //NS_LOG_FUNCTION (this << "TTLtag =1! Node "<<GetDevice()->GetNode()->GetId());
		  
		    }
		    else
		    {  
			//TTLtag = 0;//good channel
                        //NS_LOG_FUNCTION (this << "TTLtag =0! Node "<<GetDevice()->GetNode()->GetId());
		    }
		  }	
	    
		  swap(DupCountCurrent,DupCountPrev);
		  DupCountCurrent.first = p->GetUid();
		  DupCountCurrent.second = 0;
	    
	    
	      }
	      else if (p->GetUid() == DupCountCurrent.first)//old pkt
	      {
	    
		  if (DupCountCurrent.second == 9999)
		    DupCountCurrent.second = 1;
		  else
		    DupCountCurrent.second++;
	    
	      }
	      else
	      {}
	    
	  }
	  //*************************************************************************//
	  //***************channel condition determination*************************//
	  
	   if (TTLtag && tag.type == 0 && l3Header.GetTtl() < (100-TTL) && GetDevice ()->m_type != SimpleNanoDevice::NanoInterface)
	   {
		  
		TTL = 100 - l3Header.GetTtl();
                //NS_LOG_FUNCTION (this << "TTL up: Node "<<GetDevice()->GetNode()->GetId());
		
		
	   }
	   else if (!TTLtag && tag.type == 0 && l3Header.GetTtl() > (100-TTL) && GetDevice ()->m_type != SimpleNanoDevice::NanoInterface)
	   {
		TTL = 100 - l3Header.GetTtl();
                //NS_LOG_FUNCTION (this << "TTL down: Node "<<GetDevice()->GetNode()->GetId());
	     
	   }
	   else
	   {}
	  //*************************************************************************//
      
    }

}

void
FloodingNanoRoutingEntity::ForwardPacket (Ptr<Packet> p)
{
  //NS_LOG_FUNCTION (this << p << "size" << p->GetSize ());

  SenderTypeTag tag;
  p->RemovePacketTag (tag);
  NanoL3Header l3Header;
  p->RemoveHeader (l3Header);
  //NS_LOG_FUNCTION (this << l3Header);

  uint32_t dst = 0;
  uint32_t ttl = l3Header.GetTtl ();
  
  if (ttl >= 1)
    {
	  
	 
	 if (GetDevice ()->m_type == SimpleNanoDevice::NanoNode)
	    {
	      l3Header.SetTtl (ttl - 1);
	    }
	  else if (GetDevice ()->m_type == SimpleNanoDevice::NanoRouter)
	    {
	      if ((ttl - 1) > 10) l3Header.SetTtl (10);
	      else l3Header.SetTtl (ttl - 1);
	    }

	  //NS_LOG_FUNCTION (this << "new l3 header" << l3Header);
	  p->AddHeader (l3Header);

	  
          
          p->AddPacketTag (tag);
	  
	 /*
	 if (tag.type ==1 || tag.type == 2)//tag of sink pkt won't be changed
	  {
	    if (GetDevice ()->m_type == SimpleNanoDevice::NanoNode)
	      {
		  tag.type = 1;
	      }
	    else if (GetDevice ()->m_type == SimpleNanoDevice::NanoRouter)
	      {
		  tag.type = 2;
	      }
	  }
	  
	  p->AddPacketTag (tag);
	  
	  if (tag.type == 0 || tag.type == 4 || tag.type == 5)
	  {
	      
                tag.type = 2//NS_LOG_FUNCTION ("L3Forward: NodeID"<< GetDevice()->GetNode()->GetId() <<"pkt-id "<< p->GetUid() <<"pkt-size " << p->GetSize ()<<"ttl: "<<l3Header.GetTtl());
	  }
          */
	  
	  Ptr<NanoMacEntity> mac = GetDevice ()->GetMac ();
          //NS_LOG_FUNCTION ("L3Forward: NodeID"<< GetDevice()->GetNode()->GetId() <<"pkt-id "<< p->GetUid() <<"pkt-size " << p->GetSize ()<<"ttl: "<<l3Header.GetTtl());
	  mac->Send (p, dst);
    }
  else
    {
	  //NS_LOG_FUNCTION (this << "ttl expired");
    }
}

void
FloodingNanoRoutingEntity::UpdateReceivedPacketId (uint32_t id)
{
  //NS_LOG_FUNCTION (this);
  m_receivedPacketList.pop_front ();
  m_receivedPacketList.push_back (id);
}

bool
FloodingNanoRoutingEntity::CheckAmongReceivedPacket (uint32_t id)
{
  //NS_LOG_FUNCTION (this);
  for (std::list<uint32_t>::iterator it = m_receivedPacketList.begin(); it != m_receivedPacketList.end (); it++)
    {
	  //NS_LOG_FUNCTION (this << *it << id);
	  if (*it == id) return true;
    }
  return false;
}

void
FloodingNanoRoutingEntity::SetReceivedPacketListDim (int m)
{
  //NS_LOG_FUNCTION (this);
  m_receivedPacketListDim = m;
}

void
FloodingNanoRoutingEntity::L3CreteMessage ()
{
  GetDevice ()->GetMessageProcessUnit ()-> CreteMessage();
}

void
FloodingNanoRoutingEntity::L3CreteMessagePlusOne ()
{
  GetDevice ()->GetMessageProcessUnit ()-> CreteMessagePlusOne();
}

void
FloodingNanoRoutingEntity::L3CreteMessageHeader36 ()
{
  GetDevice ()->GetMessageProcessUnit ()-> CreteMessageHeader36();
}

} // namespace ns3

