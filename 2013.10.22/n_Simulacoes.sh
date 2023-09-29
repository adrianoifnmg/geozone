#!/bin/bash
rodada=0
while [ $rodada -lt 10 ]; do
  echo Rodando Simulacao $rodada
  NS_LOG=ndn.fw.V2v:ndn.V2vNetDeviceFace ./build/scenario-5consumer_1producer --run=$rodada >> log_5x1_original_ccn_$rodada.txt 2>&1
  let rodada=rodada+1
done
