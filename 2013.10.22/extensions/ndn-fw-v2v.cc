/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil -*- */
/*
 * Copyright (c) 2011 University of California, Los Angeles
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
 * Author: Alexander Afanasyev <alexander.afanasyev@ucla.edu>
 */

#include "ndn-fw-v2v.h"
#include "ndn-v2v-net-device-face.h"
#include "geo-tag.h"

#include <ns3/ndnSIM/utils/ndn-fw-hop-count-tag.h>

#include <ns3/log.h>
#include <ns3/ptr.h>
#include <ns3/assert.h>

#include <ns3/ndn-l3-protocol.h>
#include <ns3/ndn-pit-entry.h>
#include <ns3/ndn-fib-entry.h>
#include <ns3/ndn-interest.h>
#include <ns3/ndn-content-object.h>
#include <ns3/ndn-content-store.h>
#include <ns3/ndn-app-face.h>
#include <ns3/mobility-model.h>


#include <boost/foreach.hpp>

NS_LOG_COMPONENT_DEFINE ("ndn.fw.V2v");

namespace ns3 {
namespace ndn {
namespace fw {

NS_OBJECT_ENSURE_REGISTERED (V2v);

TypeId
V2v::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ndn::fw::V2v")
    .SetParent<ForwardingStrategy> ()
    .SetGroupName ("Ndn")
    .AddConstructor<V2v> ()

  ;
  return tid;
}

V2v::V2v()
{
}

V2v::~V2v ()
{
}

void
V2v::OnInterest (Ptr<Face> face,
                 Ptr<const InterestHeader> header,
                 Ptr<const Packet> origPacket)
{
  //adriano
  NS_LOG_DEBUG ("Method V2v::OnInterest");

  if (DynamicCast<AppFace> (face))
  {
      Ptr<MobilityModel> model = GetObject<MobilityModel> ();
      Vector position;
      if (model)
        {
          position = model->GetPosition ();
          GeoSrcTag tag;
          tag.SetPosition (position);
          origPacket->AddPacketTag (tag);
        }
      //adriano
      NS_LOG_INFO ("CRIACAO de Interesse em: X: " << position.x << " Y: " << position.y);
   } //enf IF

	Ptr<MobilityModel> model = GetObject<MobilityModel> ();
	Vector position;
	if (model)
	{
	  position = model->GetPosition ();
	  NS_LOG_INFO ("Chegada de Interesse na Posicao: X: " << position.x << " Y: " << position.y);
	}

	GeoTransmissionTag tagTrans;
	bool isTagTrans = origPacket->PeekPacketTag (tagTrans);
	if (isTagTrans) {
	  Vector k = tagTrans.GetPosition();
	  NS_LOG_INFO("Interesse Recebido Atraves do No: " << k.x << " " << k.y << " id no: " << k.z);
	}

  //inicio adriano - Mecanismo de Encaminhamento de Interesses Direcionado proposto por mim...
  //--------------------------------------------------------------------------------------------
  //--------------------------------------------------------------------------------------------
  //--------------------------------------------------------------------------------------------

  if (Inside_Interest_Area(GetTagOrigem(origPacket),
		  	  	  	  	   GetTagDestino(header),
		  	  	  	  	   position)) {

	  NS_LOG_INFO("Propagacao Interesse Realizada por No em X: " << position.x << " Y: " << position.y);
	  ForwardingStrategy::OnInterest (face, header, origPacket);

  } else

	  NS_LOG_INFO ("Retransmissao de Interesse Abortada no No em X: " << position.x << " Y: " << position.y);


//fim adriano - Final do Mecanismo Direcional
//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------



//Mecanismo Convencional: Apenas descomentar a linha abaixo...
//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
//    ForwardingStrategy::OnInterest (face, header, origPacket);
//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
}


void
V2v::OnData (Ptr<Face> face,
             Ptr<const ContentObjectHeader> header,
             Ptr<Packet> payload,
             Ptr<const Packet> origPacket)
{
  if (DynamicCast<AppFace> (face))
    {
      Ptr<MobilityModel> model = GetObject<MobilityModel> ();
      Vector position;
      if (model)
        {
          position = model->GetPosition ();
          GeoSrcTag tag;
          tag.SetPosition (position);
          origPacket->AddPacketTag (tag);
          // payload->AddPacketTag (tag);
        }
    }

  ForwardingStrategy::OnData (face, header, payload, origPacket);
}


bool
V2v::DoPropagateInterest (Ptr<Face> inFace,
                          Ptr<const InterestHeader> header,
                          Ptr<const Packet> origPacket,
                          Ptr<pit::Entry> pitEntry)
{
  NS_LOG_FUNCTION (this);

  int propagatedCount = 0;

  BOOST_FOREACH (const fib::FaceMetric &metricFace, pitEntry->GetFibEntry ()->m_faces.get<fib::i_metric> ())
    {
      NS_LOG_DEBUG ("Trying " << boost::cref(metricFace));
      //if (metricFace.m_status == fib::FaceMetric::NDN_FIB_RED) // all non-read faces are in the front of the list
      if (metricFace.GetStatus () == fib::FaceMetric::NDN_FIB_RED) // all non-read faces are in the front of the list
        break;

	  //inicio adriano...
      //-->Codigo transferido para a funcao GetTagOrigem...
      	  	  //GeoSrcTag tagSrc;
      	  	  //bool isTagSrc = origPacket->PeekPacketTag (tagSrc);
      	  	  //if (isTagSrc) {
    	  	  	  //Vector position = tagSrc.GetPosition();
    	  	  	  //NS_LOG_INFO("Pacote Interesse Origem em: X: " << position.x << " Y: " << position.y);
      	  	  //}

      //-->Codigo para somente para efeito de debug...
      	  	  //GeoTransmissionTag tagTrans;
      	  	  //bool isTagTrans = origPacket->PeekPacketTag (tagTrans);
      	  	  //if (isTagTrans) {
    	  	  	  //Vector position = tagTrans.GetPosition();
    	  	  	  //NS_LOG_INFO("Pacote Interesse Recebido por No em: X: " << position.x << " Y: " << position.y);
      	  	  //}

      //coleta o prefixo (e coordenadas geograficas) do interesse...
      //Código transferido para a funcao GetTagDestino...
      	  	  //Name prefixo = header->GetName();
      	  	  //std::list<std::string> interesse = prefixo.GetComponents();
      	  	  //std::list<std::string>::iterator iter = interesse.begin();
      	  	  //iter++;
   	  	  	  //std::string posicaoX_interesse = *iter;
   	  	  	  //iter++;
   	  	  	  //std::string posicaoY_interesse = *iter;
   	  	  	  //NS_LOG_INFO("Posicao (X,Y) de Interesse (" << posicaoX_interesse << "," << posicaoY_interesse << ")");

      //coleta posicao do nó...
      //somente para efeitos de debug...
      	  	  //Ptr<MobilityModel> model = GetObject<MobilityModel> ();
      	  	  //Vector actualPosition;
      	  	  //if (model)
        		//actualPosition = model->GetPosition ();
      	  	  //double_t posX_no = actualPosition.x;
      	  	  //double_t posY_no = actualPosition.y;


      //compara se posicao do nó é favoravel para propagacao do interesse
      //somente para efeitos de debug...
      	  	  //double_t posX_Interesse = atof(posicaoX_interesse.c_str());
      	  	  //if (posX_no < posX_Interesse) {
    	  	  	  //NS_LOG_INFO("Abortando Propagacao Interesse por No em X: " << posX_no << " Y: " << posY_no <<" Destino Interesse: X: " << posX_Interesse);
    	  	  //enganando o simulador -> a funcao retorna TRUE como se tivesse realmente propagado o interesse
    	  	  //return true;
      	  	  //}

      //fim adriano


      //if (!TrySendOutInterest (inFace, metricFace.m_face, header, origPacket, pitEntry))
      if (!TrySendOutInterest (inFace, metricFace.GetFace (), header, origPacket, pitEntry))
        {
    	  continue;
        }

      propagatedCount++;
    }

  NS_LOG_INFO ("Propagated to " << propagatedCount << " faces");
  return propagatedCount > 0;
}

void
V2v::DidReceiveSolicitedData (Ptr<Face> inFace,
                              Ptr<const ContentObjectHeader> header,
                              Ptr<const Packet> payload,
                              Ptr<const Packet> origPacket,
                              bool didCreateCacheEntry)
{
  Name origemPacote = header->GetName();
  std::list<std::string> XY_origem = origemPacote.GetComponents();
  std::list<std::string>::iterator iter = XY_origem.begin();
  iter++;
  std::string posicaoX_pacote = *iter;
  iter++;
  std::string posicaoY_pacote = *iter;

  NS_LOG_INFO ("RECEIVE SOLICITED DATA X: Y: " << posicaoX_pacote << " " << posicaoY_pacote);

  ForwardingStrategy::DidReceiveSolicitedData (inFace, header, payload, origPacket, didCreateCacheEntry);

  if (didCreateCacheEntry)
    {
      // initiate low priority "pushing" only for "new data packets"

      Ptr<L3Protocol> l3 = this->GetObject<L3Protocol> ();
      NS_ASSERT (l3 != 0);

      // Push interest using low-priority send method
      for (uint32_t faceId = 0; faceId < l3->GetNFaces (); faceId++)
        {
          TrySendLowPriority (l3->GetFace (faceId), origPacket);
        }
    }
}

void
V2v::DidReceiveUnsolicitedData (Ptr<Face> inFace,
                                Ptr<const ContentObjectHeader> header,
                                Ptr<const Packet> payload,
                                Ptr<const Packet> origPacket,
                                bool didCreateCacheEntry)
{
  ForwardingStrategy::DidReceiveUnsolicitedData (inFace, header, payload, origPacket, didCreateCacheEntry);
/*
  if (didCreateCacheEntry)
    {
      // initiate low priority "pushing" only for "new data packets"

      Ptr<L3Protocol> l3 = this->GetObject<L3Protocol> ();
      NS_ASSERT (l3 != 0);

      // Push interest using low-priority send method
      for (uint32_t faceId = 0; faceId < l3->GetNFaces (); faceId++)
        {
          NS_LOG_INFO("Tentando Enviar Conteudo (Nao Solicitado) " << header->GetName().GetComponents());
          TrySendLowPriority (l3->GetFace (faceId), origPacket);
        }
    }
*/
}

void
V2v::DidExhaustForwardingOptions (Ptr<Face> inFace,
                                  Ptr<const InterestHeader> header,
                                  Ptr<const Packet> origPacket,
                                  Ptr<pit::Entry> pitEntry)
{
  ForwardingStrategy::DidExhaustForwardingOptions (inFace, header, origPacket, pitEntry);

  // Try to push new packet further with lowest priority possible on all L3 faces
  BOOST_FOREACH (const fib::FaceMetric &face, pitEntry->GetFibEntry ()->m_faces)
    {
      //TrySendLowPriority (face.m_face, origPacket);
      TrySendLowPriority (face.GetFace (), origPacket);
    }
}

void
V2v::TrySendLowPriority (Ptr<Face> face, Ptr<const Packet> packet)
{
  NS_LOG_FUNCTION (boost::cref (*face));

  Ptr<V2vNetDeviceFace> v2vFace = DynamicCast<V2vNetDeviceFace> (face);
  if (v2vFace)
    {
      v2vFace->SendLowPriority (packet->Copy ());
    }
}

//inicio adriano

bool
V2v::Inside_Interest_Area(Vector origem_interesse,
		  	  	  	  	  Vector destino_interesse,
		  	  	  	  	  Vector posicao_no) {
	  //coef_linear irá definir tamanho da area em torno da
	  //reta definida pelos pontos origem_interesse e destino_interesse;
	  int coef_linear = 100;

	  //A FUNCAO ABAIXO NECESSITA DE AJUSTES!!!!
	  //CalculaRetanguloInteresse(origem_interesse, destino_interesse, coef_linear);

	  Rectangle interest_area = RetanguloInteresse(origem_interesse, destino_interesse, coef_linear);

	  if (interest_area.IsInside(posicao_no))
		  return true;
	  else
		  return false;
}


Rectangle
V2v::RetanguloInteresse(Vector o_i, //origem do interesse
						Vector d_i, //destino do interesse
		  	  	  	  	int coef_linear) {

	double x0, x1, y0, y1;
	//x0 e y0 coordenadas inferiores do retangulo de interesse
	//x1 e y1 coordenadas superiores do retangulo de interesse

	if (o_i.y >= d_i.y){
			if (o_i.x > d_i.x) {
			  x1 = o_i.x;
			  y1 = o_i.y + coef_linear;
			  x0 = d_i.x;
			  y0 = d_i.y - coef_linear;
			} else if (o_i.x < d_i.x) {
			  x1 = d_i.x;
			  y1 = o_i.y + coef_linear;
			  x0 = o_i.x;
			  y0 = d_i.y - coef_linear;
			} else {  // x origem == x destino
			  x1 = o_i.x + coef_linear;
			  y1 = o_i.y;
			  x0 = d_i.x - coef_linear;
			  y0 = d_i.y;
			}
	} else{
			if (o_i.x > d_i.x) {
			  x1 = o_i.x;
			  y1 = d_i.y + coef_linear;
			  x0 = d_i.x;
			  y0 = o_i.y - coef_linear;
			} else if (o_i.x < d_i.x) {
			  x1 = d_i.x;
			  y1 = d_i.y + coef_linear;
			  x0 = o_i.x;
			  y0 = o_i.y - coef_linear;
			} else {  // x origem == x destino
			  x1 = d_i.x + coef_linear;
			  y1 = d_i.y;
			  x0 = o_i.x - coef_linear;
			  y0 = o_i.y;
			}
	}

	NS_LOG_INFO("Area de Interesse: Inferior (" << x0 << "," << y0 << ") Superior (" << x1 << "," << y1);

	Rectangle result;
	result.xMax = x1;
	result.yMax = y1;
	result.xMin = x0;
	result.yMin = y0;

	return result;
}


Vector
V2v::GetTagOrigem(Ptr<const Packet> origPacket) {
	GeoSrcTag tagSrc;
    bool isTagSrc = origPacket->PeekPacketTag (tagSrc);
    if (isTagSrc) {
  	  Vector position = tagSrc.GetPosition();
  	  return position;
    }
}


Vector
V2v::GetTagDestino(Ptr<const InterestHeader> header) {
    Name prefixo = header->GetName();
    std::list<std::string> interesse = prefixo.GetComponents();
    std::list<std::string>::iterator iter = interesse.begin();
    iter++;
 	std::string posicaoX_interesse = *iter;
 	iter++;
 	std::string posicaoY_interesse = *iter;
 	NS_LOG_INFO("Posicao (X,Y) de Interesse (" << posicaoX_interesse << "," << posicaoY_interesse << ")");
 	Vector pos_destino;
 	pos_destino.x = atof(posicaoX_interesse.c_str());
 	pos_destino.y = atof(posicaoY_interesse.c_str());
 	pos_destino.z = 0;
 	return pos_destino;
}

/*
Rectangle
V2v::CalculaRetanguloInteresse(Vector2D o_i, //origem do interesse
		  	  	  	  	  	   Vector2D d_i, //destino do interesse
		  	  	  	  	  	   int coef_linear) {
		//ESSA FUNCAO AINDA ESTÁ FALHA!!!
		//FOI SUBSTITUIDA POR UMA VERSAO SIMPLIFICADA: RetanguloInteresse()

		//define a equacao da reta entre origem_interesse e destino_interesse...
		//equacao da reta: y = ax + b
		//a = coeficiente angular = delta_Y / delta_X
		//b = coeficiente linear
		double x0, x1, y0, y1;
		if (o_i.x >= d_i.x){
			  x1 = o_i.x;
			  y1 = o_i.y;
			  x0 = d_i.x;
			  y0 = d_i.y;
		} else {
			  x1 = d_i.x;
			  y1 = d_i.y;
			  x0 = o_i.x;
			  y0 = o_i.y;
		}
		double coef_angular = (y1-y0)/(x1-x0);
		double coef_linear_original = o_i.y - (coef_angular * o_i.x);

		//encontrando coordenadas do retangulo
		double x_min, y_min, x_max, y_max;
		y_min = (coef_angular * x0) + (coef_linear_original - coef_linear);
		x_min = (y_min - (coef_linear_original - coef_linear)) / coef_angular;
		y_max = (coef_angular * x1) + (coef_linear_original + coef_linear);
		x_max = (y_max - (coef_linear_original + coef_linear)) / coef_angular;

		NS_LOG_INFO("Area de Interesse: Inferior (" << x_min << "," << y_min << ") Superior (" << x_max << "," << y_max);

		Rectangle result;
		result.xMax = x_max;
		result.yMax = y_max;
		result.xMin = x_min;
		result.yMin = y_min;

		return result;
}
*/

//fim adriano

} // namespace fw
} // namespace ndn
} // namespace ns3
