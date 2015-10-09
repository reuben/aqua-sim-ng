//...

#include "ns3/address.h"
#include "ns3/log.h"
#include "ns3/attribute.h"
#include "ns3/simulator.h"

#include "aqua-sim-header.h"
#include "aqua-sim-packetstamp.h"
#include "aqua-sim-routing.h"
#include "aqua-sim-mac.h"

//Aqua Sim Routing

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("AquaSimRouting");
NS_OBJECT_ENSURE_REGISTERED(AquaSimRouting);


TypeId 
AquaSimRouting::GetTypeId(void)
{
	static TypeId tid = TypeId("ns3::AquaSimRouting")
		.SetParent<Object>()
		//.AddConstructor<AquaSimRouting> ()	
		//set my_addr, on-node lookup, add-ll, tracetarget, port-dmux
	;
	return tid;
}

AquaSimRouting::AquaSimRouting()
{
  NS_LOG_FUNCTION(this);
  //m_node = NULL;
  //m_pStamp = NULL;
  //m_tracetarget=NULL;		//to be implemented
  //ll(NULL), port_dmux(NULL)
}

AquaSimRouting::~AquaSimRouting()
{
  NS_LOG_FUNCTION(this);
}

/**
	* send packet p to the upper layer, i.e., port dmux
	*
	* @param p   a packet
	* */
void
AquaSimRouting::SendUp(Ptr<Packet> p)
{
	//port_dmux->recv(p); // (Handler*)NULL
	NS_LOG_FUNCTION(this << p << " : currently a dummy sendup");
	/*TODO this needs to be fully implemented with the multiplexer
			Or at least sent up for further processing 
			ie. Sync, Localization, Application driven
	*/
}

/**
	* send packet p to the lower layer
	*
	* @param p			a packet
	* @param next_hop	the next hop to route packet p
	* @param delay		packet p will be sent in time of delay
	* */
void
AquaSimRouting::SendDown(Ptr<Packet> p, Address &nextHop, Time delay)
{
	//cmh->uw_flag() = true;
	//cmh->addr_type() = NS_AF_INET;

	NS_LOG_FUNCTION(this << p << nextHop << delay);
	NS_ASSERT(p != NULL);
	

	//add header to packet
	AquaSimHeader header;
	p->PeekHeader(header);
	NS_LOG_DEBUG("Pktsize=" << header.GetSize());
	if(header.GetUId() == -1) header.SetUId(1);
	header.SetDirection(AquaSimHeader::DOWN);
	header.SetNextHop(nextHop);
	p->AddHeader(header);

	//trace here...

	//send down after given delay
	NS_LOG_FUNCTION(this << " Currently a dummy send down. delay="
			     << delay << " p=" << p);
				
	/*Note this schedule will not work, should instead call internal function once
	 * event is executed which will internal call Mac's function directly.
	 * This should most likely be a callback.
	*/
	//Simulator::Schedule(delay, &AquaSimMac::Recv, &p);
}

/**
	* check if packet p is received because of a dead loop
	*
	* @param p		a packet
	* @return		true for p experienced a dead loop, false for not
	* */

bool
AquaSimRouting::IsDeadLoop(Ptr<Packet> p) 
{
	AquaSimHeader asHeader;	
	p->PeekHeader(asHeader);
	NS_LOG_DEBUG ("SAddr=" << asHeader.GetSAddr());
	return (asHeader.GetSAddr()==m_myAddr) && (asHeader.GetNumForwards() > 0);
}

/**
	* check if this node is the source of packet p, i.e. p is generated by this node
	*
	* @param p		a packet
	* @return		true for yes, false for no
	* */
bool
AquaSimRouting::AmISrc(const Ptr<Packet> p) 
{
	AquaSimHeader asHeader;	
	p->PeekHeader(asHeader);
	NS_LOG_DEBUG ("SAddr=" << asHeader.GetSAddr());
	return (asHeader.GetSAddr()==m_myAddr) && (asHeader.GetNumForwards() == 0);
}

/**
	* check if this node is the destination of packet p, i.e. p is destined to this node
	*
	* @param p		a packet
	* @return		true for yes, false for no
	* */
bool
AquaSimRouting::AmIDst(const Ptr<Packet> p) 
{
	AquaSimHeader asHeader;	
	p->PeekHeader(asHeader);
	NS_LOG_DEBUG ("Direction=" << asHeader.GetDirection());
	return ((asHeader.GetDirection()==AquaSimHeader::UP) && (asHeader.GetDAddr() == m_myAddr));
}

/**
	* check if this node is the next hop of packetr p,
	* i.e., this node needs to forward p later on.
	*
	* @param p		a packet
	* @return		true for yes, false for no
	* */
bool
AquaSimRouting::AmINextHop(const Ptr<Packet> p) 
{
	AquaSimHeader asHeader;	
	p->PeekHeader(asHeader);
	NS_LOG_DEBUG ("NextHop=" << asHeader.GetNextHop());
	return ((asHeader.GetNextHop() == m_myAddr)|| ( asHeader.GetNextHop().IsInvalid() ));
}


}  //namespace ns3