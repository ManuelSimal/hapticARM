# hapticARM
En este repositorio se encuentra el código perteneciente al proyecto "hapticARM", que consiste en el desarrollo de un dispositivo de estimulación háptica orientado a la sustición sensorial.

Este repositorio está compuesto por dos carpetas: YetiOS_v1.0 y Unity_code.

La carpeta YetiOS_v1.0 contiene el sistema operativo sobre el cual se ha desarrollado el proyecto. Se trata de un sistema operativo basado en FreeRTOS y orientado a sensores. Ha sido desarrollado en el B105 Electronic Systems Lab. Las diferentes carpetas que lo componen son descritas en el archivo README correspondiente a la carpeta  YetiOS_v1.0.

Dentro de este sistema operativo, se ha trabajado a dos niveles. En primer lugar, se han readaptado los drivers del controlador háptico DRV2605L (existentes en un proyecto previo). Estos drivers permiten el correcto manejo de distintos actuadores hápticos. Estos drivers están recogidos en Platform/Disc-B-L475E-IOT/.

Por otro lado, se ha diseñado una librería de operaciones con cuaterniones junto con varias aplicaciones para determinar la orientación del dispositivo. Estos archivos están agrupados en la carpeta Apps. 



