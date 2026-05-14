# Motor Bringup

Este paquete contiene controladores para el sistema de motores del catamarán:

- `cmdvel_mavlink_controller`: control de propulsión y timón por MAVLink directo.
- `cable_distance_controller`: control de distancia de cable.
- `lars_height_controller`: control de altura del sistema LARS.
- Controladores antiguos o auxiliares para Mavros y timón.

## Uso

### Controlador MAVLink de propulsión y timón

```bash
ros2 launch motor_bringup cmdvel_mavlink_controller.launch.py
```

Para especificar una IP diferente:
```bash
ros2 launch motor_bringup cmdvel_mavlink_controller.launch.py udp_target_ip:=192.168.1.100
```

Publicar un comando de prueba en `/cmd_vel`:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once
```

### Controlador de distancia de cable

Lanzar el controlador:

```bash
ros2 launch motor_bringup cable_distance_controller.launch.py
```

Enviar una distancia objetivo en metros:

```bash
ros2 topic pub /cable_distance std_msgs/msg/Float32 "{data: 10.0}" --once
```

Ver la distancia actual estimada:

```bash
ros2 topic echo /cable_distance_actual
```

### Controlador de altura LARS

Lanzar el controlador:

```bash
ros2 launch motor_bringup lars_height_controller.launch.py
```

Enviar una altura objetivo en centímetros:

```bash
ros2 topic pub /lars_height std_msgs/msg/Float32 "{data: 20.0}" --once
```

Ver la altura actual estimada:

```bash
ros2 topic echo /lars_height_actual
```

### Controlador antiguo con Mavros

Si aún necesitas usar Mavros, puedes usar el controlador antiguo:

```bash
ros2 launch motor_bringup motor_bringup.launch.py
```

## Notas técnicas

- El controlador implementa el protocolo MAVLink v1.0 completo con CRC-16 CCITT
- Los mensajes RC_CHANNELS_OVERRIDE se envían a través de UDP sin confirmación (fire-and-forget)
- El nodo mantiene una secuencia de paquetes interna para el protocolo MAVLink
- Los valores de PWM están limitados a 1100-1900 µs (rango estándar)
- Los controladores Dynamixel usan el parámetro `device_name` para el puerto serie y publican el estado actual en sus topics `*_actual`.

## Solución de problemas

Si no recibe comandos el autopiloto:
1. Verifica que el IP y puerto sean correctos
2. Asegúrate de que el firewall permite UDP 14550
3. Revisa los logs del nodo: `ros2 topic echo /cmd_vel` para verificar que llegan comandos
4. Aumenta el nivel de debug: agrega `log_level:=DEBUG` al lanzar el nodo

Si no responde un controlador Dynamixel:
1. Verifica que el adaptador USB esté conectado y que el usuario tenga permisos sobre `/dev/ttyUSB*`.
2. Comprueba que los motores estén alimentados.
3. Si el puerto no es el esperado, cambia `device_name` en el launch file correspondiente.
