/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2015 UWSN Lab at the University of Connecticut
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
 * Author: Robert Martin <robert.martin@engr.uconn.edu>
 */

#ifndef AQUA_SIM_NET_DEVICE_H
#define AQUA_SIM_NET_DEVICE_H

#include "ns3/mobility-model.h"
#include "ns3/net-device.h"
#include "ns3/traced-callback.h"
#include "ns3/address.h"      //could be updated to support own unique address type for uwsn

#include "aqua-sim-phy.h"
#include "aqua-sim-mac.h"
#include "aqua-sim-channel.h"
#include "aqua-sim-node.h"

namespace ns3 {

/**
 * \Underwater net device structure.
 *
 * A basic underwater net device structure. Ported from UWSN Lab's Aqua-Sim on NS2.
 */


class AquaSimMobilityPattern;  //Additional mobility patterns
class AquaSimPhy;
class AquaSimChannel;
class AquaSimMac;
class AquaSimNode;
//class AquaSimApp;
//class AquaSimRouting;
class MobilityModel;

class AquaSimNetDevice : public NetDevice
{
public:
  AquaSimNetDevice ();
  ~AquaSimNetDevice ();
  static TypeId GetTypeId (void);  

  //attach
  void SetPhy (Ptr<AquaSimPhy> phy);
  void SetMac (Ptr<AquaSimMac> mac);
  void SetChannel (Ptr<AquaSimChannel> channel);
  //void SetApp (Ptr<AquaSimApp> app);
  
  Ptr<AquaSimPhy> GetPhy (void);
  Ptr<AquaSimMac> GetMac (void);
  //Ptr<AquaSimApp> GetApp (void);
        //Not currently implemented  
  
  virtual void DoDispose (void);
  virtual void DoInitialize (void);

  void ForwardUp (Ptr<Packet> packet, Ptr<MobilityModel> src, Ptr<MobilityModel> dst);

  //inherited functions from NetDevice class
  virtual void AddLinkChangeCallback (Callback<void> callback);
  virtual Address GetAddress (void) const;
  virtual Address GetBroadcast (void) const;
  virtual Ptr<AquaSimChannel> GetChannel (void);
  virtual uint32_t GetIfIndex (void) const;
  virtual uint16_t GetMtu (void) const;
  virtual Address GetMulticast (Ipv4Address multicastGroup) const;
  virtual Address GetMulticast (Ipv6Address addr) const;
  virtual Ptr<AquaSimNode> GetNode (void);
  virtual bool IsBridge (void) const;
  virtual bool IsBroadcast (void) const;
  virtual bool IsLinkUp (void) const;
  virtual bool IsMulticast (void) const;
  virtual bool IsPointToPoint (void) const;
  virtual bool NeedsArp (void) const;
  virtual bool Send (Ptr<Packet> packet, const Address &dest, uint16_t protocolNumber);
  virtual bool SendFrom (Ptr<Packet> packet, const Address &source, 
                         const Address &dest, uint16_t protocolNumber);
  virtual void SetAddress (Address address);
  virtual void SetIfIndex (const uint32_t index);
  virtual bool SetMtu (const uint16_t mtu);
  virtual void SetNode (Ptr<AquaSimNode> node);
  virtual void SetPromiscReceiveCallback (PromiscReceiveCallback cb);
  virtual void SetReceiveCallback (ReceiveCallback cb);
  virtual bool SupportsSendFrom (void) const;



  /*    //sink and general node functions & variables below...
  //Sink
  int ClearSinkStatus ();
  int SetSinkStatus ();		//variable???
  int GetSinkStatus () {return sinkStatus_; }  

  inline bool FailureStatus () { return failureStatus_; }
  inline double FailurePro () { return failurePro; }
  inline double FailureStatusPro () { return failureStatusPro_; }

  void SetCarrierSense (bool f)

  int m_sinkStatus;
  bool m_failureStatus;  //1 if node fails, 0 otherwise
  double m_failurePro;
  double m_failureStatusPro;
 
*/ 


private:
  
  void CompleteConfig (void);
  
  Ptr<AquaSimNode> m_node;
  Ptr<AquaSimPhy> m_phy;
  Ptr<AquaSimMac> m_mac;
  //Ptr<AquaSimApp> m_app;
  Ptr<AquaSimChannel> m_channel;

  NetDevice::ReceiveCallback m_forwardUp;
  bool m_configComplete;

  //m_clear for dispose?? to clear all layers from net-device side.

};  // class AquaSimNetDevice


}  // namespace ns3

#endif /* AQUA_SIM_NET_DEVICE_H */