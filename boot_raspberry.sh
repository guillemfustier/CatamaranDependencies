#!/bin/bash

source entorno_docker.sh


#ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14551@





# Usa argumentos $1 y $2 si existen, sino usa valores por defecto
LAT_ORIGIN=${1:-39.99446582}
LON_ORIGIN=${2:--0.07405792}

echo "Iniciando con origen GPS: Lat=$LAT_ORIGIN, Lon=$LON_ORIGIN"

ros2 launch catamaran_sensors mavlink_tf.launch.py origin_lat:=$LAT_ORIGIN origin_lon:=$LON_ORIGIN