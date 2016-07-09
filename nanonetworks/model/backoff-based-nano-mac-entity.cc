/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013,2014 TELEMATICS LAB, DEE - Politecnico di Bari
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



#include "backoff-based-nano-mac-entity.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/packet.h"
#include "simple-nano-device.h"
#include "nano-mac-queue.h"
#include "nano-spectrum-phy.h"
#include "nano-mac-header.h"
#include "ns3/seq-ts-header.h"
#include "ns3/simulator.h"

NS_LOG_COMPONENT_DEFINE ("BackoffBasedNanoMacEntity");

namespace ns3 {


NS_OBJECT_ENSURE_REGISTERED (BackoffBasedNanoMacEntity);

TypeId BackoffBasedNanoMacEntity::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::BackoffBasedNanoMacEntity")
    .SetParent<NanoMacEntity> ();
  return tid;
}


BackoffBasedNanoMacEntity::BackoffBasedNanoMacEntity ()
{
  SetMacQueue (CreateObject <NanoMacQueue> ());
  SetDevice (0);
  Simulator::Schedule (Seconds(0.001), &NanoMacEntity::CheckForNeighbors, this);
}


BackoffBasedNanoMacEntity::~BackoffBasedNanoMacEntity ()
{
}

void 
BackoffBasedNanoMacEntity::DoDispose (void)
{
  NanoMacEntity::DoDispose ();
}

void
BackoffBasedNanoMacEntity::DoSendPacket ()//if the first one doesn't backoff, the following ones would rarely backoff!Unless:no neighbor!
{
  //NS_LOG_FUNCTION (this);
  Ptr<NanoSpectrumPhy> phy = GetDevice ()->GetPhy ();

  //NS_LOG_FUNCTION (this << "m_neighbors.size ()" << m_neighbors.size ());
  
  if (m_neighbors.size ())
    {
	  srand ( time(NULL) );
	  Ptr<Packet> p = (m_queue.front ())->Copy ();
	  
	  m_queue.pop_front ();//dequeue

	  NanoMacHeader header;
	  p->RemoveHeader (header);
	  uint32_t dst = header.GetDestination ();
	  //NS_LOG_FUNCTION (this << "check dst" << header);

	  if (dst == GetDevice ()->GetNode ()->GetId ())//work as a relay
		{
		  //NS_LOG_FUNCTION (this << "select new dst");
		  int i = rand () %m_neighbors.size();//0~neighbour_size
		  dst = m_neighbors.at (i).first;//vector<pair<id,type>>
		  header.SetDestination (dst);
		}
	  p->AddHeader (header);
	  //NS_LOG_FUNCTION (this << "new header" << header);
	  phy->StartTx (p);//send 

	  if (m_queue.size () > 0)
		{
		  double backoff = rand () % 100000;
		  //************check neighbor existance************//
		  /*bool neighbor_probe = false;
		  for (int i=0;i<m_neighbors.size();i++)
		  {
		    if (m_neighbors[i].first == 10)
		      neighbor_probe = true;
		    
		  }*/
		  //************end*********************************//
		  
		  //if (neighbor_probe)//probe the neighbour of node #
		  NS_LOG_FUNCTION(this<<"Have neighbor! Backoff! NodeID: "<<GetDevice()->GetNode()->GetId()<< "pkt-id: "<<p->GetUid()<<"backoff: "<<backoff);
		  Simulator::Schedule (PicoSeconds (backoff), &BackoffBasedNanoMacEntity::DoSendPacket, this);
		}
    }
  else if (m_queue.size () > 0)
    {		
	  double backoff = rand () % 10;
	  //NS_LOG_FUNCTION (this << "try again after (ms)" << backoff);
	  NS_LOG_FUNCTION(this<<"No neighbor! Backoff: "<<backoff);
	  Simulator::Schedule (MilliSeconds (backoff), &BackoffBasedNanoMacEntity::DoSendPacket, this);
    }
  else
    {}
}

void
BackoffBasedNanoMacEntity::Send (Ptr<Packet> p)
{
  //NS_LOG_FUNCTION (this << p);

  NanoMacHeader header;
  uint32_t src = GetDevice ()->GetNode ()->GetId ();
  uint32_t dst = 0;
  header.SetSource (src);
  header.SetDestination (dst);

  //NS_LOG_FUNCTION (this << "mac header" << header);
  //NS_LOG_FUNCTION (this << "pkt-size:"<<p->GetSize ());
  //NS_LOG_FUNCTION (this << "queue-size"<<m_queue.size ());
  p->AddHeader (header);

  m_queue.push_back (p);

  if (m_queue.size () == 1)
	{
	  Simulator::Schedule (Seconds (0.0), &BackoffBasedNanoMacEntity::DoSendPacket, this);
	}
}

void
BackoffBasedNanoMacEntity::Send (Ptr<Packet> p, uint32_t dst)
{
  //NS_LOG_FUNCTION (this << p << dst);
  //NS_LOG_FUNCTION (this << "pkt-size:"<<p->GetSize ());
  NanoMacHeader header;
  uint32_t src = GetDevice ()->GetNode ()->GetId ();
  header.SetSource (src);
  header.SetDestination (dst);

  //NS_LOG_FUNCTION (this << "mac header" << header);
  //NS_LOG_FUNCTION (this << "queue-size"<<m_queue.size ());
  p->AddHeader (header);

  m_queue.push_back (p);//enqueue!!!
  if (m_queue.size () >= 2)
  NS_LOG_FUNCTION("Enqueue! NodeID: "<<GetDevice()->GetNode()->GetId()<< "pkt-id: "<<p->GetUid()<<"queue-size "<<m_queue.size ());
  if (m_queue.size () == 1)//why???::wait until the back-offed pkts are gone (only one queue space?)
	{
	  Simulator::Schedule (Seconds (0.0), &BackoffBasedNanoMacEntity::DoSendPacket, this);
	  //NS_LOG_FUNCTION("Send!");
	}
}

void
BackoffBasedNanoMacEntity::Receive (Ptr<Packet> p)
{
  //NS_LOG_FUNCTION (this);
}

} // namespace ns3
