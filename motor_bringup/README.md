# Motor Bringup - MAVLink Controller

## Descripción

Este paquete contiene controladores para el sistema de motores del catamarán. Se ha reemplazado el controlador basado en Mavros (`cmdvel_motor_controller`) con un nuevo controlador que se comunica directamente por MAVLink a través de UDP: `cmdvel_mavlink_controller`.

## Cambios principales

### Nuevo: `cmdvel_mavlink_controller`

El nuevo controlador **no requiere Mavros** y envía los comandos directamente al autopiloto a través del puerto UDP 14550 usando el protocolo MAVLink v1.0.

#### Características:
- Suscripción a `/cmd_vel` (mensajes `geometry_msgs/Twist`)
- Conversión directa de velocidad lineal a PWM de motores
- Conversión de velocidad angular a ángulo de timón
- Envío de comandos RC_CHANNELS_OVERRIDE por UDP

#### Parámetros configurables:
- `udp_target_ip` (default: "127.0.0.1"): IP del autopiloto/simulador
- `udp_target_port` (default: 14550): Puerto UDP del autopiloto
- `target_system_id` (default: 1): ID del sistema MAVLink destino
- `target_component_id` (default: 1): ID del componente MAVLink destino

#### Mapeos de control:
- **Motor izquierdo (RC1)**: `linear.x` mapea a PWM 1100-1900 (1500 = neutral)
- **Motor derecho (RC3)**: negación de `linear.x` para movimiento diferencial
- **Timón (RC4)**: `angular.z` mapea a PWM 1100-1900 (-40° a +40° en radianes)

## Uso

### Opción 1: Lanzar el controlador MAVLink (recomendado)

```bash
ros2 launch motor_bringup cmdvel_mavlink_controller.launch.py
```

Para especificar una IP diferente:
```bash
ros2 launch motor_bringup cmdvel_mavlink_controller.launch.py udp_target_ip:=192.168.1.100
```

### Opción 2: Controlador antiguo (Mavros)

Si aún necesitas usar Mavros, puedes usar el controlador antiguo:

```bash
ros2 launch motor_bringup motor_bringup.launch.py
```

## Requisitos

- ROS 2
- El paquete `geometry_msgs`
- Socket UDP estándar (incluido en la mayoría de sistemas Unix/Linux)

**No requiere:**
- Mavros
- Librería externa de MAVLink (el protocolo se implementa de forma nativa)

## Compilación

```bash
cd ~/CatamaranDependencies
colcon build --packages-select motor_bringup
```

## Notas técnicas

- El controlador implementa el protocolo MAVLink v1.0 completo con CRC-16 CCITT
- Los mensajes RC_CHANNELS_OVERRIDE se envían a través de UDP sin confirmación (fire-and-forget)
- El nodo mantiene una secuencia de paquetes interna para el protocolo MAVLink
- Los valores de PWM están limitados a 1100-1900 µs (rango estándar)

## Solución de problemas

Si no recibe comandos el autopiloto:
1. Verifica que el IP y puerto sean correctos
2. Asegúrate de que el firewall permite UDP 14550
3. Revisa los logs del nodo: `ros2 topic echo /cmd_vel` para verificar que llegan comandos
4. Aumenta el nivel de debug: agrega `log_level:=DEBUG` al lanzar el nodo
