#!/bin/bash

# 1. Exportar la ruta de tus librerías PIP personalizadas
export PYTHONPATH=$PYTHONPATH:/root/persistent_ws/pip_dependencies/

# 2. Activar el entorno base de ROS2 (por si acaso)
source /opt/ros/jazzy/setup.bash

# 3. Activar tu workspace compilado
if [ -f "/root/persistent_ws/CatamaranDependencies/install/setup.bash" ]; then
    source /root/persistent_ws/CatamaranDependencies/install/setup.bash
fi

echo "✅ Entorno cargado: PIP, ROS2 y Workspace listos."