<!DOCTYPE html>
<html lang="es">
</head>
<body>
  <header>
    <h1>SoC Agro-Telemetría para John Deere</h1>
    <p><em>Demostración de firmware y prototipo de adquisición de datos en tiempo real</em></p>
  </header>
  <section id="vision-general">
    <h2>1. Visión General</h2>
    <p>
      Este proyecto implementa un <strong>sistema de telemetría agrícola</strong> compuesto por:
    </p>
    <ul>
      <li>Un microcontrolador <code>STM32F103RB</code> en un entorno <em>FreeRTOS</em>, encargado de leer valores analógicos (potenciómetros o sensores) y controlar una pantalla LCD y un teclado matricial.</li>
      <li>Una <strong>Raspberry Pi</strong> (o dispositivo similar) que recibe los datos vía UART desde el STM32, procesa la información en Python y genera registros locales o envía la telemetría a un servidor en la nube.</li>
      <li>Una interfaz de visualización local (dashboard web simple) para supervisar valores en tiempo real y comprobar el correcto funcionamiento del sistema.</li>
    </ul>
    <p>
      En el video <em>funcionamiento.mp4</em> se muestra el prototipo montado sobre un protoboard y una placa de desarrollo STM32:  
      <strong>3 potenciómetros</strong> (simulando tres sensores de distinto tipo),  
      una <strong>pantalla LCD 16×2</strong> que exhibe el valor medido (en voltios o unidades arbitrarias),  
      y un <strong>teclado matricial 4×4</strong> que sirve para seleccionar modos o cambiar parámetros en tiempo real.
    </p>
  </section>
  
  <section id="prototipo-en-accion">
    <h2>2. Protótipo en Acción</h2>
    <p>
      A continuación se describen las escenas más relevantes observadas en el video:
    </p>
    <ul>
      <li>
        <strong>Lectura de valores analógicos:</strong> El STM32 convierte la señal analógica de cada potenciómetro a un valor de ADC. En la pantalla LCD se visualiza, por ejemplo, “<code>Key: 0 - ADC: 1.78</code>” indicando que, al presionar la tecla 0, el sistema muestra la lectura actual de voltaje (1.78 V en este caso).
        <br>
      </li>
      <li>
        <strong>Interacción mediante teclado matricial:</strong> Cada vez que se presiona una tecla del keypad 4×4, el firmware cambia de “canal” o “modo” de lectura. El LCD actualiza instantáneamente el valor correspondiente al potenciómetro seleccionado.  
        <br>
        </li>
      <li>
        <strong>Transmisión de datos a Raspberry Pi:</strong> En un tramo del video se aprecia la Raspberry Pi (o laptop) recibiendo datos por UART y mostrando en pantalla (terminal o script Python) los valores de ADC en tiempo real.  
        <br>
       </li>
      <li>
        <strong>Integración con hardware auxiliar:</strong> Se identifican los potenciómetros conectados al ADC del STM32, el LCD en interfaz paralelo, y el keypad en puertos GPIO. Además, se observa cómo el cable USB proporciona alimentación y conexión serial al PC para flashear el MCU y visualizar datos.  
      </li>
    </ul>
  </section>
</body>
