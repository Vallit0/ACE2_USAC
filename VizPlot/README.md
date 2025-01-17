# VizPlot
VizPlot es una librería para Arduino diseñada para:
- Visualizar gráficos ASCII en tiempo real.
- Gestionar logs con distintos niveles de severidad.

## Instalación
1. Descarga el repositorio.
2. Copia la carpeta `VizPlot` en tu directorio de `libraries`.

## Uso
```cpp
#include <VizPlot.h>
VizPlot logger("Temperature Log", 15);

void setup() {
    Serial.begin(9600);
    logger.begin(&Serial);
}

void loop() {
    int temperature = analogRead(A0);
    logger.addData(temperature);
    logger.plotLine();
    delay(1000);
}
```