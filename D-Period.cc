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
 *
 *
 * run:
 *
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/internet-module.h"
#include "ns3/propagation-loss-model.h"//m_outPHYTX
#include <iostream>
#include <math.h>
#include "ns3/global-route-manager.h"
#include "ns3/nano-mac-entity.h"
#include "ns3/nano-mac-queue.h"
#include "ns3/nano-spectrum-channel.h"
#include "ns3/nano-spectrum-phy.h"
#include "ns3/nano-spectrum-signal-parameters.h"
#include "ns3/nano-helper.h"
#include "ns3/nano-spectrum-value-helper.h"
#include "ns3/simple-nano-device.h"
#include "ns3/nanointerface-nano-device.h"
#include "ns3/nanorouter-nano-device.h"
#include "ns3/nanonode-nano-device.h"
#include "ns3/backoff-based-nano-mac-entity.h"
#include "ns3/seq-ts-header.h"
#include "ns3/ts-ook-based-nano-spectrum-phy.h"
#include "ns3/mobility-model.h"
#include "ns3/message-process-unit.h"
#include "ns3/transparent-nano-mac-entity.h"
#include "ns3/random-nano-routing-entity.h"
#include "ns3/flooding-nano-routing-entity.h"


NS_LOG_COMPONENT_DEFINE ("D-channel");

using namespace ns3;



void Run(Ptr<FriisPropagationLossModel> FSPL, int nbNanoNodes, int nbNanoRouters, int nbNanoGateways, double txRangeNanoNodes, double txRangeNanoRouter, int macType, int l3Type, int seed);

void PrintTXEvents(Ptr<OutputStreamWrapper> stream, int, int,int,int);
void PrintRXEvents(Ptr<OutputStreamWrapper> stream, int, int, int, int, double,double);
void PrintRXTTL(Ptr<OutputStreamWrapper> stream, int, int);
void PrintPHYTXEvents(Ptr<OutputStreamWrapper> stream, int, int);
void PrintPHYRXEvents(Ptr<OutputStreamWrapper> stream, int);//check received pkts at all nodes(pkt size)
void PrintPHYCOLLEvents(Ptr<OutputStreamWrapper> stream, int, int);
void PrintPTxEvents(Ptr<OutputStreamWrapper> stream, int, int);
void PrintNodeDegree(Ptr<OutputStreamWrapper> stream, NetDeviceContainer devs);
void PrintKfDup(Ptr<OutputStreamWrapper> stream, NetDeviceContainer devs);
void PrintKfTTL(Ptr<OutputStreamWrapper> stream, NetDeviceContainer devs);
void PrintKfRx(Ptr<OutputStreamWrapper> stream, Ptr<MessageProcessUnit> mpu);
void PrintSimulationTime(double duration);
static void PrintPosition(std::ostream *os, std::string foo, NetDeviceContainer devs);
void PrintMemoryUsage(void);
//void DynamicChannel(Ptr<NanoSpectrumChannel> c, double k);
void DynamicChannel(double k);
void DynamicBKratio(double x);

int main(int argc, char *argv[]) {
  
  
        //LogComponentEnable("NanoSpectrumChannel",LOG_LEVEL_ALL);
	//LogComponentEnable("TransparentNanoMacEntity",LOG_LEVEL_ALL);
	//LogComponentEnable("NanoMacEntity",LOG_LEVEL_ALL);
	//LogComponentEnable("BackoffBasedNanoMacEntity",LOG_LEVEL_ALL);
	//LogComponentEnable("NanoMacQueue",LOG_LEVEL_ALL);
	//LogComponentEnable("MessageProcessUnit",LOG_LEVEL_ALL);
	//LogComponentEnable("FloodingNanoRoutingEntity",LOG_LEVEL_ALL);
	//LogComponentEnable("TsOokBasedNanoSpectrumPhy",LOG_LEVEL_ALL);
	//LogComponentEnable("NanoSpectrumChannel",LOG_LEVEL_ALL);
	//LogComponentEnable("MessageProcessUnit",LOG_LEVEL_ALL);
	//LogComponentEnable("SimpleNanoDevice",LOG_LEVEL_ALL);
	
	int nbNanoNodes = 20; 
	int nbNanoRouters = 0;
	int nbNanoGateways = 1;
	double txRangeNanoNodes = 0.01;
	double txRangeNanoRouter = 0.01;
	int macType = 1;
	int l3Type = 3;
	int seed = 1;
	double min_distance = 0.00001;
	double frequency = 1e12;
	double speed = 3e8;
	//*********THIS IS THE ABSORPTION COEFFICIENT******************************//
	
	//*************build a FSPL model for nodes to use******************************//
	Ptr<FriisPropagationLossModel> FSPL = CreateObject <FriisPropagationLossModel> ();
	
	
	//******************************************************************************//

	CommandLine cmd;
	cmd.AddValue("seed", "seed", seed);
	cmd.AddValue("nbNanoNodes", "nbNanoNodes", nbNanoNodes);
	cmd.AddValue("nbNanoRouters", "nbNanoRouters", nbNanoRouters);
	cmd.AddValue("nbNanoGateways", "nbNanoGateways", nbNanoGateways);
	cmd.AddValue("txRangeNanoNodes", "txRangeNanoNodes", txRangeNanoNodes);
	cmd.AddValue("txRangeNanoRouter", "txRangeNanoRouter", txRangeNanoNodes);
	cmd.AddValue("macType", "macType", macType);
	cmd.AddValue("l3Type", "l3Type", l3Type);
	cmd.AddValue("frequency", "frequency", frequency);
	cmd.Parse(argc, argv);
	
	
	FSPL -> SetLambda (frequency, speed);
	FSPL -> SetMinDistance(min_distance);
	Run(FSPL, nbNanoNodes, nbNanoRouters, nbNanoGateways, txRangeNanoNodes, txRangeNanoRouter, macType, l3Type, seed);

	return 0;
}

void Run(Ptr<FriisPropagationLossModel> FSPL, int nbNanoNodes, int nbNanoRouters, int nbNanoGateways, double txRangeNanoNodes, double txRangeNanoRouter, int macType, int l3Type, int seed)
{
        
	//****************SET K(f)*************************//
	
        //*************************************************//
	std::cout << "START SIMULATION WITH: "<<
			"\n\t lambda " << FSPL->GetLambda() <<
			"\n\t nbNanoNodes " << nbNanoNodes <<
			"\n\t nbNanoRouters " << nbNanoRouters <<
			"\n\t nbNanoGateways " << nbNanoGateways <<
			"\n\t txRangeNanoNodes " << txRangeNanoNodes <<
			"\n\t txRangeNanoRouter "<< txRangeNanoRouter <<
			"\n\t macType [1->Transparent; 2->BackoffBased] "<< macType <<
			"\n\t l3Type [1->Flooding; 2->Random Routing] "<< l3Type <<
			"\n\t seed " << seed << std::endl;
	
			
	
	//timers
	Time::SetResolution(Time::FS);//resolution not unit!!!
	double duration = 600;

	//layout details
	double xrange = 10;//still have to manually change location limitation
	double yrange = 10;//still have to manually change location limitation
	double zrange = 0.0001;

	//physical details
	double pulseEnergy = 100e-12;
	double pulseDuration = 100;
	double pulseInterval = 10000;        
	//double powerTransmission = pulseEnergy / pulseDuration;
	double powerTransmission = -5.85+30; //dBm:average power of a 1 PJ pulse

	//helper definition
	NanoHelper nano;
	//nano.EnableLogComponents ();
	


	// Tracing
	AsciiTraceHelper asciiTraceHelper;
	/*
	std::stringstream file_outTX_s;
	file_outTX_s << "RES_TX" << "_N_" << nbNanoNodes << "_R_" << nbNanoRouters << "_G_" << nbNanoGateways <<
			"_nTxRange_" << txRangeNanoNodes << "_macType_" << macType << "_l3Type_" << l3Type << "_seed_" << seed;
	std::stringstream file_outRX_s;
	file_outRX_s << "RES_RX" << "_N_" << nbNanoNodes << "_R_" << nbNanoRouters << "_G_" << nbNanoGateways <<
			"_nTxRange_" << txRangeNanoNodes << "_macType_" << macType << "_l3Type_" << l3Type << "_seed_" << seed;
	std::stringstream file_outCorrectRX_s;
	std::stringstream file_outPHYTX_s;
	file_outPHYTX_s << "RES_PHYTX" << "_N_" << nbNanoNodes << "_R_" << nbNanoRouters << "_G_" << nbNanoGateways <<
			"_nTxRange_" << txRangeNanoNodes << "_macType_" << macType << "_l3Type_" << l3Type << "_seed_" << seed;
	std::stringstream file_outPHYCOLL_s;
	file_outPHYCOLL_s << "RES_PHYCOLL" << "_N_" << nbNanoNodes << "_R_" << nbNanoRouters << "_G_" << nbNanoGateways <<
			"_nTxRange_" << txRangeNanoNodes << "_macType_" << macType << "_l3Type_" << l3Type << "_seed_" << seed;
	std::stringstream file_outPTx_s;
	file_outPTx_s << "RES_PTx" << "_N_" << nbNanoNodes << "_R_" << nbNanoRouters << "_G_" << nbNanoGateways <<
			"_nTxRange_" << txRangeNanoNodes << "_macType_" << macType << "_l3Type_" << l3Type << "_seed_" << seed;
	std::stringstream file_outDupPkt_s;
	file_outDupPkt_s << "RES_DupPkt" << "_N_" << nbNanoNodes << "_R_" << nbNanoRouters << "_G_" << nbNanoGateways <<
			"_nTxRange_" << txRangeNanoNodes << "_macType_" << macType << "_l3Type_" << l3Type << "_seed_" << seed;		
	*/		
	
	std::stringstream file_outTX_s;
	file_outTX_s << "RES_TX_"<<"N_"<<nbNanoNodes<<"_l3Type_"<<l3Type<<"_seed_"<<seed;
	std::stringstream file_outRX_s;
	file_outRX_s << "RES_RX_"<<"N_"<<nbNanoNodes<<"_l3Type_"<<l3Type<<"_seed_"<<seed;
        std::stringstream file_outRXTTL_s;
	file_outRXTTL_s << "RES_RXTTL_"<<"N_"<<nbNanoNodes<<"_l3Type_"<<l3Type<<"_seed_"<<seed;
	std::stringstream file_outCorrectRX_s;
	std::stringstream file_outPHYTX_s;
	file_outPHYTX_s << "RES_PHYTX_"<<"N_"<<nbNanoNodes<<"_l3Type_"<<l3Type<<"_seed_"<<seed;
	std::stringstream file_outPHYCOLL_s;
	file_outPHYCOLL_s << "RES_PHYCOLL_"<<"N_"<<nbNanoNodes<<"_l3Type_"<<l3Type<<"_seed_"<<seed;
	std::stringstream file_outPTx_s;
	file_outPTx_s << "RES_PTx_"<<"N_"<<nbNanoNodes<<"_l3Type_"<<l3Type<<"_seed_"<<seed;
        /*
	std::stringstream file_outNodeDegree_s;
	file_outNodeDegree_s << "RES_NodeDegree_"<<"_N_"<<nbNanoNodes<<"_l3Type_"<<l3Type<<"_seed_"<<seed;
	std::stringstream file_outKfDup_s;
	file_outKfDup_s << "RES_KfDup_"<<"_N_"<<nbNanoNodes<<"_l3Type_"<<l3Type<<"_seed_"<<seed;
	std::stringstream file_outKfTTL_s;
	file_outKfTTL_s << "RES_KfTTL_"<<"_N_"<<nbNanoNodes<<"_l3Type_"<<l3Type<<"_seed_"<<seed;
	std::stringstream file_outKfRx_s;
	file_outKfRx_s << "RES_KfRx_"<<"_N_"<<nbNanoNodes<<"_l3Type_"<<l3Type<<"_seed_"<<seed;
        */
	std::stringstream file_outPHYRX_s;
	file_outPHYRX_s << "RES_PHYRX_"<<"N_"<<nbNanoNodes<<"_l3Type_"<<l3Type<<"_seed_"<<seed;
	

	std::string file_outTX = file_outTX_s.str();
	std::string file_outRX = file_outRX_s.str();
        std::string file_outRXTTL = file_outRXTTL_s.str();
	std::string file_outPHYTX = file_outPHYTX_s.str();
	std::string file_outPHYCOLL = file_outPHYCOLL_s.str();
	std::string file_outPTx = file_outPTx_s.str();
        /*
	std::string file_outNodeDegree = file_outNodeDegree_s.str();
	std::string file_outKfDup = file_outKfDup_s.str();
	std::string file_outKfTTL = file_outKfTTL_s.str();
	std::string file_outKfRx = file_outKfRx_s.str();
        */
	std::string file_outPHYRX = file_outPHYRX_s.str();

	
	Ptr<OutputStreamWrapper> streamTX = asciiTraceHelper.CreateFileStream(
			file_outTX);
	Ptr<OutputStreamWrapper> streamRX = asciiTraceHelper.CreateFileStream(
			file_outRX);
        Ptr<OutputStreamWrapper> streamRXTTL = asciiTraceHelper.CreateFileStream(
			file_outRXTTL);
	Ptr<OutputStreamWrapper> streamPHYTX = asciiTraceHelper.CreateFileStream(
			file_outPHYTX);
	Ptr<OutputStreamWrapper> streamPHYCOLL = asciiTraceHelper.CreateFileStream(
			file_outPHYCOLL);
	Ptr<OutputStreamWrapper> streamPTx = asciiTraceHelper.CreateFileStream(
			file_outPTx);
        /*
	Ptr<OutputStreamWrapper> streamNodeDegree = asciiTraceHelper.CreateFileStream(
			file_outNodeDegree);
	Ptr<OutputStreamWrapper> streamKfDup = asciiTraceHelper.CreateFileStream(
			file_outKfDup);
	Ptr<OutputStreamWrapper> streamKfTTL = asciiTraceHelper.CreateFileStream(
			file_outKfTTL);
	Ptr<OutputStreamWrapper> streamKfRx = asciiTraceHelper.CreateFileStream(
			file_outKfRx);
        */
	Ptr<OutputStreamWrapper> streamPHYRX = asciiTraceHelper.CreateFileStream(
			file_outPHYRX);

	//network definition
	NodeContainer n_routers;
	NodeContainer n_gateways;
	NodeContainer n_nodes;
	NetDeviceContainer d_gateways;
	NetDeviceContainer d_nodes;
	NetDeviceContainer d_routers;

	n_gateways.Create(nbNanoGateways);
	d_gateways = nano.Install(n_gateways, NanoHelper::nanointerface);
	n_routers.Create(nbNanoRouters);
	d_routers = nano.Install(n_routers, NanoHelper::nanorouter);
	n_nodes.Create(nbNanoNodes);
	d_nodes = nano.Install(n_nodes, NanoHelper::nanonode);


	//mobility
	
	//*****************to generate fixed topology*********************//
        //srand(time(NULL));	
        SeedManager::SetSeed(seed);
	//****************************************************************//
	MobilityHelper mobility;
	mobility.SetMobilityModel ("ns3::GaussMarkovMobilityModel",
			"Bounds", BoxValue (Box (0, xrange, 0, yrange, 0, zrange)),
			"TimeStep", TimeValue (Seconds (0.001)),
			"Alpha", DoubleValue (0),
			//**************no mobility***********************************************	   
			"MeanVelocity", StringValue ("ns3::UniformRandomVariable[Min=0|Max=0]"),
			"MeanDirection", StringValue ("ns3::UniformRandomVariable[Min=0|Max=0]"),
			"MeanPitch", StringValue ("ns3::UniformRandomVariable[Min=0|Max=0]"),
			"NormalVelocity", StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]"),
			"NormalDirection", StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]"),
			"NormalPitch", StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]"));
			//*****************mobiliy*****************************************************
			/*"MeanVelocity", StringValue ("ns3::UniformRandomVariable[Min=0.2|Max=0.2]"),
			"MeanDirection", StringValue ("ns3::UniformRandomVariable[Min=0|Max=0]"),
			"MeanPitch", StringValue ("ns3::UniformRandomVariable[Min=0.05|Max=0.05]"),
			"NormalVelocity", StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]"),
			"NormalDirection", StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=0.2|Bound=0.4]"),
			"NormalPitch", StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=0.2|Bound=0.4]"));*/
        	
        mobility.SetPositionAllocator ("ns3::RandomBoxPositionAllocator",
			"X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=10]"),//RandomVariableValue (UniformVariable (0, xrange)),
			"Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=10]"),//RandomVariableValue (UniformVariable (0, yrange)),
			"Z", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=0]"));//RandomVariableValue (UniformVariable (0, zrange)));
        
        /**
        mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
			"rho", StringValue ("0.02"),//radius,
			"X", StringValue ("0.02"),//x_center,
			"Y", StringValue ("0.02"));//y_center;
        **/
	mobility.Install (n_nodes);//position of nanorouters & gateways are set in the "for" loops below
	mobility.Install (n_routers);
	std::string logFile = "nanonodeposition.log";
	std::string logFile2 = "nanorouterposition.log";
	std::ofstream os;
	std::ofstream os2;
	os.open(logFile.c_str());
	os2.open(logFile2.c_str());
	
	Simulator::Schedule(Seconds(0.001), &PrintPosition, &os, logFile, d_nodes);
	Simulator::Schedule(Seconds(10), &PrintPosition, &os2, logFile2, d_nodes);
	
	//random variable generation
	//srand(time(NULL));
	SeedManager::SetSeed(seed);
	UniformVariable random;
	//***********make Kf random******************//
	//Kf=Kf*random.GetValue(0.0,5.0);
	//Kf=0;
	//std::cout<<"RealTimeKf: "<<Kf<<std::endl;
	//*******************************************//
	

	//protocol stack
	for (int x = 0; x < nbNanoGateways; x++) {
		
	  
	  
		Ptr<NanoInterfaceDevice> interface = d_gateways.Get(x)->GetObject<
				NanoInterfaceDevice> ();
		Ptr<ConstantPositionMobilityModel> m = CreateObject<
				ConstantPositionMobilityModel> ();

		m->SetPosition(Vector(xrange/2, yrange/2, zrange/2));//box
                //m->SetPosition(Vector(0.02, 0.02, 0));//circle

		nano.AddMobility(interface->GetPhy(), m);

		Ptr<NanoRoutingEntity> routing;
		if (l3Type == 1 || l3Type == 3) {
			Ptr<FloodingNanoRoutingEntity> routing2 = CreateObject<
					FloodingNanoRoutingEntity> ();
			routing = routing2;
		} else if (l3Type == 2) {
			Ptr<RandomNanoRoutingEntity> routing2 = CreateObject<
					RandomNanoRoutingEntity> ();
			routing = routing2;
		} else {
		}
		routing->SetDevice(interface);
		interface->SetL3(routing);

		Ptr<NanoMacEntity> mac;
		if (macType == 1) {
			Ptr<TransparentNanoMacEntity> mac2 = CreateObject<
					TransparentNanoMacEntity> ();
			mac = mac2;
		} else if (macType == 2) {
			Ptr<BackoffBasedNanoMacEntity> mac2 = CreateObject<
					BackoffBasedNanoMacEntity> ();
			mac = mac2;
		} else {
		}
		mac->SetDevice(interface);
		interface->SetMac(mac);
		//***********************set fspl******************************//
		//interface->GetPhy()->SetChannel(ChannelMabs);
		interface->GetPhy()->GetChannel()->AddPropagationLossModel(FSPL);
		//interface->GetPhy()->GetChannel()->m_Kf=Kf;
		//*************************************************************//
		//**********************set Kf*********************************//
		
		//**************************************************************
		interface->GetPhy()->SetTxPower(powerTransmission);
		interface->GetPhy()->SetTransmissionRange(txRangeNanoRouter);
		interface->GetPhy()->GetObject<TsOokBasedNanoSpectrumPhy> ()->SetPulseDuration(
				FemtoSeconds(pulseDuration));
		interface->GetPhy()->GetObject<TsOokBasedNanoSpectrumPhy> ()->SetPulseInterval(
				FemtoSeconds(pulseInterval));

		interface->GetPhy()->TraceConnectWithoutContext("outPHYTX",
				MakeBoundCallback(&PrintPHYTXEvents, streamPHYTX));
		interface->GetPhy()->TraceConnectWithoutContext("outPHYCOLL",
				MakeBoundCallback(&PrintPHYCOLLEvents, streamPHYCOLL));
                interface->GetPhy()->TraceConnectWithoutContext("outPTx",
				MakeBoundCallback(&PrintPTxEvents, streamPTx));
		
		
	}

	for (int x = 0; x < nbNanoRouters; x++) {
		
		Ptr<NanoRouterDevice> router = d_routers.Get(x)->GetObject<
				NanoRouterDevice> ();
		Ptr<MobilityModel> m = n_routers.Get(x)->GetObject<MobilityModel>();
		//CreateObject<
				//ConstantPositionMobilityModel> ();
		//m->SetPosition(Vector(xrange/2, yrange/2, zrange/2));
		nano.AddMobility(router->GetPhy(), m);
		

		Ptr<NanoRoutingEntity> routing;
		if (l3Type == 1 || l3Type == 3) {
			Ptr<FloodingNanoRoutingEntity> routing2 = CreateObject<
					FloodingNanoRoutingEntity> ();
			routing = routing2;
		} else if (l3Type == 2) {
			Ptr<RandomNanoRoutingEntity> routing2 = CreateObject<
					RandomNanoRoutingEntity> ();
			routing = routing2;
		} else {
		}
		routing->SetDevice(router);
		router->SetL3(routing);

		Ptr<NanoMacEntity> mac;
		if (macType == 1) {
			Ptr<TransparentNanoMacEntity> mac2 = CreateObject<
					TransparentNanoMacEntity> ();
			mac = mac2;
		} else if (macType == 2) {
			Ptr<BackoffBasedNanoMacEntity> mac2 = CreateObject<
					BackoffBasedNanoMacEntity> ();
			mac = mac2;
		} else {
		}
		mac->SetDevice(router);
		router->SetMac(mac);
		//***********************set fspl******************************//
		//router->GetPhy()->SetChannel(ChannelMabs);
		router->GetPhy()->GetChannel()->AddPropagationLossModel(FSPL);
		//router->GetPhy()->GetChannel()->m_Kf=Kf;
		//***********************set fspl*****************************//
		//**********************set Kf*********************************//
		
		//**************************************************************
		router->GetPhy()->SetTxPower(powerTransmission);
		router->GetPhy()->SetTransmissionRange(txRangeNanoRouter);
		router->GetPhy()->GetObject<TsOokBasedNanoSpectrumPhy> ()->SetPulseDuration(
				FemtoSeconds(pulseDuration));
		router->GetPhy()->GetObject<TsOokBasedNanoSpectrumPhy> ()->SetPulseInterval(
				FemtoSeconds(pulseInterval));

		router->GetPhy()->TraceConnectWithoutContext("outPHYTX",
				MakeBoundCallback(&PrintPHYTXEvents, streamPHYTX));
		router->GetPhy()->TraceConnectWithoutContext("outPHYCOLL",
				MakeBoundCallback(&PrintPHYCOLLEvents, streamPHYCOLL));
		router->GetPhy()->TraceConnectWithoutContext("outPTx",
				MakeBoundCallback(&PrintPTxEvents, streamPTx));


	}

	for (uint32_t i = 0; i < d_nodes.GetN(); i++) {
		Ptr<MobilityModel> m = n_nodes.Get(i)->GetObject<MobilityModel> ();
		nano.AddMobility(
				d_nodes.Get(i)->GetObject<NanoNodeDevice> ()->GetPhy(), m);
		Ptr<NanoNodeDevice> dev = d_nodes.Get(i)->GetObject<NanoNodeDevice> ();

		Ptr<NanoRoutingEntity> routing;
		if (l3Type == 1 || l3Type == 3) {
			Ptr<FloodingNanoRoutingEntity> routing2 = CreateObject<
					FloodingNanoRoutingEntity> ();
			routing = routing2;
		} else if (l3Type == 2) {
			Ptr<RandomNanoRoutingEntity> routing2 = CreateObject<
					RandomNanoRoutingEntity> ();
			routing = routing2;
		} else {
		}
		routing->SetDevice(dev);
		dev->SetL3(routing);

		Ptr<NanoMacEntity> mac;
		if (macType == 1) {
			Ptr<TransparentNanoMacEntity> mac2 = CreateObject<
					TransparentNanoMacEntity> ();
			mac = mac2;
		} else if (macType == 2) {
			Ptr<BackoffBasedNanoMacEntity> mac2 = CreateObject<
					BackoffBasedNanoMacEntity> ();
			mac = mac2;
		} else {
		}
		mac->SetDevice(dev);
		dev->SetMac(mac);
		//***********************set fspl******************************//
		//dev->GetPhy()->SetChannel(ChannelMabs);
		dev->GetPhy()-> GetChannel() -> AddPropagationLossModel(FSPL);
		//dev->GetPhy()->GetChannel()->m_Kf=Kf;
		//***********************set fspl*****************************//
		//**********************set Kf*********************************//
		
		//**************************************************************
		dev->GetPhy()->SetTransmissionRange(txRangeNanoNodes);
		dev->GetPhy()->SetTxPower(powerTransmission);
		dev->GetPhy()->GetObject<TsOokBasedNanoSpectrumPhy> ()->SetPulseDuration(
				FemtoSeconds(pulseDuration));
		dev->GetPhy()->GetObject<TsOokBasedNanoSpectrumPhy> ()->SetPulseInterval(
				FemtoSeconds(pulseInterval));

		dev->GetPhy()->TraceConnectWithoutContext("outPHYTX",
				MakeBoundCallback(&PrintPHYTXEvents, streamPHYTX));
                dev->GetL3()->GetObject<FloodingNanoRoutingEntity>()->TraceConnectWithoutContext("outRXTTL",
				MakeBoundCallback(&PrintRXTTL, streamRXTTL));
		dev->GetPhy()->TraceConnectWithoutContext("outPHYCOLL",
				MakeBoundCallback(&PrintPHYCOLLEvents, streamPHYCOLL));
		dev->GetPhy()->TraceConnectWithoutContext("outPTx",
				MakeBoundCallback(&PrintPTxEvents, streamPTx));
		dev->GetPhy()->TraceConnectWithoutContext("outPHYRX",
				MakeBoundCallback(&PrintPHYRXEvents, streamPHYRX));

	}





	//application
	
	double packetInterval = 60;//seconds

	for (int i = 0; i < nbNanoNodes; i++) {

		Ptr<MessageProcessUnit> mpu = CreateObject<MessageProcessUnit> ();
		mpu->SetDevice(d_nodes.Get(i)->GetObject<SimpleNanoDevice> ());
		d_nodes.Get(i)->GetObject<SimpleNanoDevice> ()->SetMessageProcessUnit(
				mpu);
		
                //SeedManager::SetSeed(rand());
                mpu->SetInterarrivalTime(packetInterval);
		//mpu->SetInterarrivalTime(random.GetValue(0.001,packetInterval));

		//double startTime = random.GetValue(0.011, 0.1);
                //double startTime = random.GetValue(0.011, packetInterval);
                double startTime = 0;
		//std::cout<<"Node "<<i<<"StartTime: "<<startTime<<std::endl;
		//double startTime=0.1;
		//Simulator::Schedule(Seconds(startTime),
		//		&MessageProcessUnit::CreteMessage, mpu);
		mpu->TraceConnectWithoutContext("outTX",
				MakeBoundCallback(&PrintTXEvents, streamTX));
		mpu->TraceConnectWithoutContext("outRX",
				MakeBoundCallback(&PrintRXEvents, streamRX));
	}
        
        
	for (int i = 0; i < nbNanoGateways; i++) {
		Ptr<MessageProcessUnit> mpu = CreateObject<MessageProcessUnit> ();
		mpu->SetDevice(d_gateways.Get(i)->GetObject<SimpleNanoDevice> ());
		d_gateways.Get(i)->GetObject<SimpleNanoDevice> ()->SetMessageProcessUnit(
				mpu);
		mpu->SetInterarrivalTime(packetInterval);
		SeedManager::SetSeed(seed);
//double startTime = random.GetValue(0.01, 0.011);
		double startTime = 0.00001;
		
                if (l3Type == 3)
                {
                  Simulator::Schedule(Seconds(startTime),&MessageProcessUnit::CreteMessageHeader36, mpu);//36-beacon 43-polling
               
                }		
                
		mpu->TraceConnectWithoutContext("outTX",
				MakeBoundCallback(&PrintTXEvents, streamTX));
		mpu->TraceConnectWithoutContext("outRX",
				MakeBoundCallback(&PrintRXEvents, streamRX));
	}
        
	
	//********************************Build Dynamic channel**********************************
        double KfInterval = (duration)/10-0.1; 
        double KfArray[] = {0.38679154596,1.05145221671,2.79155644373,4.94244355627,6.68254778329,7.34720845404,6.68254778329,4.94244355627,2.79155644373,1.05145221671,0.38679154596};
        double ratio_BK = 1.0;
        bool D_channel = true;
        //double k = 4.94244355627;
        //Simulator::Schedule(Seconds(0.0), &DynamicChannel, k);
	
	if (D_channel)
	{  
	  int j = 0;
	
	  for (double i = 0.0; i <= duration; i += KfInterval, j++)
	  {
	    
            ratio_BK = random.GetValue(0.0,1.0);
            Simulator::Schedule(Seconds(i), &DynamicChannel, KfArray[j]);//(KfArray[j]-2)*9.73+11.14
            Simulator::Schedule(Seconds(i), &DynamicBKratio, ratio_BK);//change backhaul ratio
	    //Simulator::Schedule(Seconds(i), &PrintNodeDegree, streamNodeDegree, d_nodes);
	    //Simulator::Schedule(Seconds(i), &PrintKfDup, streamKfDup, d_nodes);
	    //Simulator::Schedule(Seconds(i), &PrintKfTTL, streamKfTTL, d_nodes);
	    //Simulator::Schedule(Seconds(i), &PrintKfRx, streamKfRx, d_gateways.Get(0)->GetObject<SimpleNanoDevice> ()->GetMessageProcessUnit());
	  }
	}
	//********************************

	Simulator::Stop(Seconds(duration+1));
	//Simulator::Stop(Seconds(0.5));
	Simulator::Schedule(Seconds(0.), &PrintSimulationTime, duration);
	Simulator::Schedule(Seconds(duration), &PrintMemoryUsage);

	Simulator::Run();
	Simulator::Destroy();
	
}





static void PrintPosition(std::ostream *os, std::string foo,
		NetDeviceContainer devs) {
	int num = devs.GetN();
	for (int i = 0; i < num; i++) {
		Ptr<SimpleNanoDevice> d = devs.Get(i)->GetObject<SimpleNanoDevice> ();
		Ptr<MobilityModel> m = d->GetPhy()->GetMobility();
		Vector pos = m->GetPosition();
                /**
		*os << Simulator::Now().GetSeconds() << " " << d->GetNode()->GetId()
								<< " " <<d->m_type<< " " << pos.x << " " << pos.y << " " << pos.z << std::endl;
**/
                *os << d->GetNode()->GetId()<<" "<<pos.x << " " << pos.y << std::endl;
	}
	//Simulator::Schedule(Seconds(0.1), &PrintPosition, os, foo, devs);//for mobility record
}

void PrintNodeDegree(Ptr<OutputStreamWrapper> stream, NetDeviceContainer devs)
{
	
	int num = devs.GetN();
	int avg;
	int sum = 0;
	      for (int i = 0; i < num; i++) {
		    Ptr<SimpleNanoDevice> d = devs.Get(i)->GetObject<SimpleNanoDevice> ();
		    sum += d->GetMac ()->m_neighbors.size();
	      }
	  avg = sum/num;
	*stream->GetStream() << avg<<std::endl;
	
	
	/*
	int count;
	Ptr<SimpleNanoDevice> d = devs.Get(2)->GetObject<SimpleNanoDevice> ();
	count = d->GetMac ()->m_neighbors.size();
	*stream->GetStream() <<count<<std::endl;
	*/
}

void PrintKfDup(Ptr<OutputStreamWrapper> stream, NetDeviceContainer devs)
{
	
	int num = devs.GetN();
	int count = 0;
	
	int sum = 0;
	      for (int i = 0; i < num; i++) {
		    Ptr<SimpleNanoDevice> d = devs.Get(i)->GetObject<SimpleNanoDevice> ();
		    if (d->GetL3 ()-> GetObject<FloodingNanoRoutingEntity>() -> DupCount)
		    //if (d->GetL3 ()-> GetObject<FloodingNanoRoutingEntity>() -> TTLmax)
		    {  
		      sum += d->GetL3 ()-> GetObject<FloodingNanoRoutingEntity>() -> DupCount;
		      //sum += d->GetL3 ()-> GetObject<FloodingNanoRoutingEntity>() -> TTLmax;
		      count++;
		     
		    }
		    d->GetL3 ()-> GetObject<FloodingNanoRoutingEntity>() -> DupCount = 0;  
	      }
	  
	if (count)
	  sum = sum/count;
	else
	  sum = 0;
	  
	*stream->GetStream() << sum<<std::endl;
	
}

void PrintKfTTL(Ptr<OutputStreamWrapper> stream, NetDeviceContainer devs)
{
	
	double num = (double)devs.GetN();
	double avg;
	double sum = 0;

	      for (int i = 0; i < num; i++) {
		    Ptr<SimpleNanoDevice> d = devs.Get(i)->GetObject<SimpleNanoDevice> ();
		    sum += d->GetL3 ()-> GetObject<FloodingNanoRoutingEntity>() -> TTL;		    
	      }
	  avg = sum/num;
	*stream->GetStream() << avg<<std::endl;
	
	
	/*
	int count;
	Ptr<SimpleNanoDevice> d = devs.Get(2)->GetObject<SimpleNanoDevice> ();
	count = d->GetMac ()->m_neighbors.size();
	*stream->GetStream() <<count<<std::endl;
	*/
}

void PrintKfRx(Ptr<OutputStreamWrapper> stream, Ptr<MessageProcessUnit> mpu)
{
    
    
      *stream->GetStream()<<mpu->RxCount<<std::endl;
      mpu->RxCount = 0;
  
}




void PrintMemoryUsage(void) {
	system(
			"ps aux | grep build/scratch/health-care | head -1 | awk '{print $1, $4, $10}'");
}


void PrintTXEvents(Ptr<OutputStreamWrapper> stream, int id, int src, int type, int size) {
	*stream->GetStream() << Simulator::Now().GetSeconds() << " source-ID "<< src << " pkt-ID " << id << " node-type " <<type<<" pkt-size "<<size<<std::endl;
}

void PrintRXEvents(Ptr<OutputStreamWrapper> stream, int id, int size, int src,
		int thisNode, double delay, double ratio_backhaul) {
	*stream->GetStream()<<Simulator::Now().GetSeconds()<<" Gate-ID " << thisNode << " pkt-ID " << id << " pkt-size " << size << " pkt-delay " << delay
			<< " source-ID "<<src<<" BKratio "<<ratio_backhaul<<std::endl;
}

void PrintRXTTL(Ptr<OutputStreamWrapper> stream, int id, int T) {
	*stream->GetStream()<<Simulator::Now().GetSeconds()<<" pkt-ID " << id << " TTL " << T <<std::endl;
}

void PrintPHYCOLLEvents(Ptr<OutputStreamWrapper> stream, int id, int thisNode) {
	*stream->GetStream()<<Simulator::Now().GetSeconds() << " node-ID "<<thisNode << " pkt-ID " << id << std::endl;
}

void PrintPHYTXEvents(Ptr<OutputStreamWrapper> stream, int id, int src) {
	*stream->GetStream()<<Simulator::Now().GetSeconds()<<" "<<src <<" "<< id << std::endl;
}

void PrintPHYRXEvents(Ptr<OutputStreamWrapper> stream, int size) {
	*stream->GetStream()<<Simulator::Now().GetSeconds()<<" "<< size << std::endl;
}

void PrintPTxEvents(Ptr<OutputStreamWrapper> stream, int id, int pktid) {
	*stream->GetStream()<<Simulator::Now().GetSeconds()<<" "<< id << " "<<pktid << std::endl;
}


void PrintSimulationTime(double duration) {

	double percentage = (100. * Simulator::Now().GetSeconds()) / duration;
	std::cout << "*** " << percentage << " *** " << std::endl;

	double deltaT = duration/10;
	int t = Simulator::Now().GetSeconds() / deltaT;

	double nexttime = deltaT * (t+1);
	Simulator::Schedule(Seconds(nexttime), &PrintSimulationTime, duration);
}


void DynamicChannel(double k)
{
  
  NanoSpectrumChannel::m_Kf = k;
  
}

void DynamicBKratio(double x)
{
        MessageProcessUnit::ratio_backhaul = x;
}

