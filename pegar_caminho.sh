#!/bin/bash

MIR_IP="10.83.131.110"
AUTH_TOKEN="ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=="

# 1️⃣ Obter a missão atual
MISSION_ID=$(curl -s -X GET "http://$MIR_IP/api/v2.0.0/status" \
     -H "Authorization: Basic $AUTH_TOKEN" \
     -H "Accept-Language: en_US" \
     -H "accept: application/json" | jq -r '.mission_queue_id')

echo "Missão atual: $MISSION_ID"

# 2️⃣ Obter todas as ações da missão
POSITIONS=$(curl -s -X GET "http://$MIR_IP/api/v2.0.0/mission_queue/$MISSION_ID/actions" \
     -H "Authorization: Basic $AUTH_TOKEN" \
     -H "Accept-Language: en_US" \
     -H "accept: application/json" | jq -r '.[] | select(.action_type=="move") | .parameters.position')

echo "Posições na missão: $POSITIONS"

# 3️⃣ Obter coordenadas de cada posição
echo "Coordenadas da missão:"
for POS_ID in $POSITIONS; do
    COORDS=$(curl -s -X GET "http://$MIR_IP/api/v2.0.0/positions/$POS_ID" \
        -H "Authorization: Basic $AUTH_TOKEN" \
        -H "Accept-Language: en_US" \
        -H "accept: application/json" | jq -r '"X: \(.pos_x), Y: \(.pos_y), Orientação: \(.orientation)"')
    echo "$COORDS"
done
