# MASN V1.2 (HT-RA62) — nodo Reticulum solar autónomo

*[English version](README.md)*

Soporte en este firmware para la placa **MASN** (*Mesh Autonomous Solar Node*), con el que la placa funciona como **nodo [Reticulum](https://reticulum.network/) autónomo**: un nodo de transporte que recibe, valida, enruta y retransmite paquetes por LoRa sin ordenador. También puede funcionar como un RNode normal controlado por Reticulum desde un ordenador.

## Qué es MASN

MASN es una PCB abierta para montar un nodo LoRa solar autónomo. La diseñó Daniel P. Costas a partir de los encuentros en Bricolabs (A Coruña), basándose en el diseño fakeTec. Originalmente se pensó para Meshtastic.

- Artículo y guía de montaje: <https://danielpcostas.dev/es/masn-nodo-solar-meshtastic-que-cualquiera-puede-montar/>
- Hardware (Gerber, STL): <https://github.com/danielcharrua/masn-meshtastic-autonomous-solar-node>. Licencia CERN OHL-S v2.

Este firmware es para la **V1.2 con módulo Heltec HT-RA62**. Las variantes con Waveshare Core1262 y la MASN V2.0 no están probadas.

## Componentes

| Componente | Pieza | Uso en este firmware |
|---|---|---|
| Microcontrolador | nice!nano (nRF52840) | Ejecuta el firmware y el stack microReticulum |
| Radio LoRa | Heltec HT-RA62 (SX1262, 868 MHz), TCXO 1,8 V | Interfaz LoRa |
| Cargador solar | CN3791 MPPT | No lo controla el firmware |
| Batería | LiPo 3,7 V con BMS (p. ej. 4400 mAh) | — |
| Panel solar | 5–6 V | — |
| Sensor de corriente/tensión | INA3221 (I2C 0x40) | Batería, placa solar y consumo en la página NomadNet y en `masn_setup.py power` |
| Sensor ambiental | BME280 (I2C 0x76) | Detectado, **todavía no se usa** |
| Antena | 868 MHz, con cable UFL→SMA | — |

### Pines (verificados en hardware)

| Señal | nRF52840 | Notas |
|---|---|---|
| SX1262 NSS (CS) | P1.13 (45) | |
| SCK / MOSI / MISO | P1.11 (43) / P1.15 (47) / P0.02 (2) | SPIM2 |
| RESET | P0.09 (9) | Pin NFC, funciona como GPIO |
| **BUSY** | **P0.29 (29)** | **No es P0.20** |
| DIO1 (IRQ) | P0.10 (10) | Pin NFC, funciona como GPIO |
| RXEN | P0.17 (17) | En TX lo conmuta DIO2 del SX1262 |
| **VCC externo** | **P0.13 (13)** | Alimenta el HT-RA62. **Tiene que estar en HIGH** |
| TCXO | DIO3 del SX1262 a 1,8 V | |

Coincide con la variante `nrf52_promicro_diy_tcxo` de Meshtastic ("NRF52 Pro-micro DIY").

## Compilar y flashear

Necesitas [PlatformIO](https://platformio.org/) y `rnodeconf` (paquete `rns` de Python). La nice!nano debe tener el bootloader 0.8 o superior, igual que para Meshtastic.

```sh
pio run -e masn-ht-ra62                                    # compilar
pio run -e masn-ht-ra62 -t upload --upload-port /dev/ttyACM0
```

`upload` hace tres cosas: flashea por DFU USB, aprovisiona la ROM (producto/modelo `0x53`) y graba el hash del firmware, sin el cual la placa no activa la radio.

- `rnodeconf` todavía no conoce el modelo `0x53`. Por eso el hash se graba directamente por KISS. En una placa ya aprovisionada, `rnodeconf --rom` muestra un `KeyError: 83` que no tiene consecuencias.
- El aprovisionamiento de una placa nueva, nunca aprovisionada, **no está verificado** todavía.
- Si flasheas por otro medio, graba el hash después con `python masn/masn_setup.py hash`.

Después de flashear, ejecuta `python masn/masn_setup.py` (ver más abajo).

> Si después de cambiar el código el hash no cambia, PlatformIO ha reutilizado un `.zip` antiguo. Bórralo con `rm .pio/build/masn-ht-ra62/*.{hex,zip,bin}` y vuelve a compilar.

## Configuración: `masn_setup.py`

`masn/masn_setup.py` es el programa de configuración de una MASN ya flasheada. Habla con la placa por USB usando el protocolo KISS de RNode y el subsistema de aprovisionamiento del firmware, así que no necesita ninguna otra herramienta. `rnodeconf --tnc` y `--firmware-hash` abortan con el modelo MASN (`0x53`); este programa los sustituye.

```sh
pip install pyserial rns lxmf          # lxmf solo hace falta para la visibilidad en mapas
python masn/masn_setup.py              # asistente interactivo
```

El asistente localiza la nice!nano por USB, espera a que arranque Reticulum en la placa y pregunta paso a paso. Propone el valor actual por defecto: Intro lo mantiene y `-` vacía un texto opcional.

1. **Radio.** Muestra los valores por defecto (869,618 MHz, 62,5 kHz, SF5, CR4/5, 7 dBm) y pregunta si quieres cambiarlos. Tienen que coincidir con los del resto de tu red. Avisa de los límites de EU868: 500 mW PRA con un ciclo de trabajo del 10 % en 869,40–869,65 MHz, y normalmente 25 mW al 1 % en el resto.
2. **Modo.** Nodo de transporte Reticulum autónomo (la configuración se guarda y la placa arranca como nodo al recibir alimentación, sin USB) o RNode para un ordenador.
3. **Nombre y página NomadNet.** El nombre del nodo y si se publica una página NomadNet. Si se publica, pide una descripción de una línea y un contacto del operador.
4. **Visibilidad en mapas ([rmap.world](https://rmap.world)).** Si la activas, pide la ubicación en una sola línea, `latitud, longitud` en grados decimales con punto (se puede pegar desde un mapa, p. ej. `42.8125, -1.6458`), después la altitud, si la ubicación se muestra también en la página NomadNet, una dirección LXMF del operador (opcional) y el intervalo de anuncio (6 h por defecto; rmap penaliza a los nodos que anuncian más de 3 veces por hora). La página NomadNet tiene identidad propia (`NOMADNET_OWN_IDENTITY`), distinta de la del nodo, así que rmap muestra un solo marcador por nodo (la interfaz LoRa) y lista la página solo en *Services*. rmap borra los nodos que llevan 7 días sin anunciarse.
5. **Resumen y confirmación.** Lo graba todo, reinicia la placa, comprueba que vuelve a arrancar y muestra la dirección NomadNet del nodo y su identidad de transporte. Guarda las respuestas en un perfil (`masn-<id>.json`); `--profile fichero.json` las reutiliza como valores por defecto, por ejemplo para configurar varios nodos.

Subcomandos para usar desde scripts:

| Comando | Qué hace |
|---|---|
| `status` | Estado de la placa (diagnóstico `0x60`): `ONLINE`, `HW`, frecuencia, potencia, pines |
| `show` | Configuración guardada del nodo (nombre, página, ubicación, discovery) y sus direcciones |
| `hash` | Graba el hash de `.pio/build/masn-ht-ra62/*.bin` (solo si flasheas sin `pio … -t upload`) |
| `tnc` / `rnode` | Cambia de modo, con las opciones de radio `--freq --bw --sf --cr --txp` |
| `power` | Batería, placa solar y consumo del nodo según el INA3221 (diagnóstico `0x6C`) |

Opciones comunes: `--port` (por defecto detecta la nice!nano) y `--profile`. El idioma de la interfaz sigue a `LANG` (español o inglés).

Para ver el nodo funcionando, abre el puerto como consola serie (`pio device monitor -b 115200` o `screen /dev/ttyACM0 115200`). Busca líneas `Valid announce for …` y `Rebroadcasting announce …`. Cierra la consola antes de volver a ejecutar `masn_setup.py`, porque solo un programa puede usar el puerto a la vez.

### Qué publica el nodo

- **Página NomadNet** (`nomadnetwork.node`): nombre, descripción, contacto, ubicación (si está activada), parámetros de radio y tiempo encendido, seguidos de las páginas de estadísticas del firmware. El sitio se anuncia al arrancar y después en cada intervalo de anuncio.
- **Interface discovery** (`rnstransport.discovery.interface`, solo si es visible): el mismo anuncio que envía `rnsd` para una interfaz con `discoverable = yes`. Lleva el nombre, la ubicación, los parámetros de radio y el sello de prueba de trabajo que exigen los mapas. El sello lo calcula `masn_setup.py` en el ordenador (en el nRF52 tardaría minutos) y se guarda en la placa, que lo reenvía 2 minutos después de arrancar y luego en cada intervalo. Un nodo solo LoRa llega al mapa únicamente si algún nodo de transporte enlaza tu red LoRa con Internet.

## Configuración manual

No hace falta usar `masn_setup.py`. Hay dos formas de configurar el nodo a mano.

### Opción A: nodo autónomo, con comandos KISS directos

La placa acepta tramas KISS estándar de RNode por USB a 115200 baudios: `C0 <comando> <datos> C0`, donde `C0` y `DB` dentro de los datos se escapan como `DB DC` y `DB DD`. Ten en cuenta dos detalles:

- Todo comando necesita al menos un byte de datos. `CMD_CONF_SAVE` se envía como `53 00`; sin el `00` el firmware lo ignora.
- Mientras el puerto esté abierto hay que leerlo (o vaciarlo). Si el ordenador lo deja abierto sin leer, se llena el búfer USB, el firmware se bloquea al escribir su log y el watchdog reinicia la placa a los 60 s.

| Comando | Datos | Significado |
|---|---|---|
| `01` | 4 bytes, big endian | Frecuencia en Hz |
| `02` | 4 bytes, big endian | Ancho de banda en Hz |
| `03` | 1 byte | Potencia TX en dBm |
| `04` | 1 byte | Spreading factor |
| `05` | 1 byte | Coding rate (5–8) |
| `06` | `01` | Encender la radio |
| `53` | `00` | Guardar la configuración de radio: la placa arranca como nodo de transporte |
| `54` | `00` | Borrarla: la placa arranca como RNode |
| `55` | `F8` | Reiniciar |
| `86` | msgpack | Petición de aprovisionamiento (`[op, seq, payload]`; SetState = 5, Commit = 6) |

El nombre, la página NomadNet y la ubicación son campos de aprovisionamiento del namespace `100`:

| Id | Campo | Tipo |
|---|---|---|
| 4 | NomadNet Enabled | bool |
| 5 | NomadNet Name | texto, ≤ 63 |
| 20 | Site Description | texto, ≤ 160 |
| 21 | Site Contact | texto, ≤ 64 |
| 22 / 23 / 24 | Latitude / Longitude / Altitude | float |
| 25 | Show Location (en la página NomadNet) | bool |
| 26 | Discovery Enabled | bool |
| 27 | Discovery Data (anuncio con su sello, ver `build_discovery_app_data()` en `masn_setup.py`) | bytes |
| 28 | Announce Interval (minutos, 30–1440) | int |

Ejemplo (Python con `pyserial` y `rns`), probado en la placa:

```python
import serial, struct, time
from RNS.vendor import umsgpack

s = serial.Serial("/dev/ttyACM0", 115200, timeout=0.1)

def kiss(cmd, data):
    data = data.replace(b"\xdb", b"\xdb\xdd").replace(b"\xc0", b"\xdb\xdc")
    s.write(b"\xc0" + bytes([cmd]) + data + b"\xc0")
    time.sleep(1)
    s.reset_input_buffer()              # mantener el puerto vaciado

# 1. Nombre y página NomadNet: aprovisionamiento SetState + Commit (namespace 100)
kiss(0x86, umsgpack.packb([5, 1, {3: {100: {5: "Mi nodo MASN",
                                            20: "Nodo solar en el monte",
                                            21: "LXMF: <tu dirección>"}}}]))
kiss(0x86, umsgpack.packb([6, 2, {}]))

# 2. Radio, guardar como nodo autónomo y reiniciar
kiss(0x01, struct.pack(">I", 869618000))   # frecuencia (Hz)
kiss(0x02, struct.pack(">I", 62500))       # ancho de banda (Hz)
kiss(0x03, bytes([7]))                     # potencia TX (dBm)
kiss(0x04, bytes([5]))                     # spreading factor
kiss(0x05, bytes([5]))                     # coding rate
kiss(0x06, b"\x01")                        # encender la radio
kiss(0x53, b"\x00")                        # guardar: arranca como nodo de transporte
kiss(0x55, b"\xf8")                        # reiniciar
```

El aprovisionamiento solo responde cuando Reticulum ha arrancado en la placa, hasta un minuto después de encenderla. La consola web RNode Console (`Release/console.html`, ver [docs/Provisioning.md](../docs/Provisioning.md)) usa el mismo protocolo y muestra estos campos en *Transport Config → General*; no se ha probado con la MASN.

### Opción B: como RNode para Reticulum en un ordenador

Pon la placa en modo RNode (`masn_setup.py rnode`, o el comando `54 00` y reiniciar) y añádela a `~/.reticulum/config` como cualquier RNode. Probado con `rnsd` 1.5.7:

```ini
[interfaces]
  [[MASN]]
    type = RNodeInterface
    enabled = yes
    port = /dev/ttyACM0
    frequency = 869618000
    bandwidth = 62500
    txpower = 7
    spreadingfactor = 5
    codingrate = 5
```

En este caso el encaminamiento lo hace el ordenador (`enable_transport = yes` en `[reticulum]`). Si ese ordenador además tiene Internet, es la forma más sencilla de aparecer en [rmap.world](https://rmap.world): añade a la interfaz `discoverable = yes`, `discovery_name = …`, `latitude = …`, `longitude = …`, `height = …` y `announce_interval = 360`, e instala `lxmf` (`pip install lxmf`). Esta última parte (`discoverable`) sale de la documentación de RNS y de rmap, y no se ha probado con la MASN.

## Herramientas de verificación

| Script | Para qué sirve |
|---|---|
| `masn_osc_test.py` | Arranque del SX1262 paso a paso: escritura SPI, reset, TCXO, STDBY_XOSC y PLL. No emite RF. |
| `masn_sdr_txtest.py` | Emite una portadora CW y un paquete LoRa y los mide con un RTL-SDR (necesita `rtl_sdr` y `numpy`). |
| `masn_rxmon.py` | Monitor de recepción: RSSI, SNR y cabecera Reticulum de cada paquete. |

Necesitan la placa en modo RNode (`masn_setup.py rnode`). Las pruebas que emiten solo se ejecutan a 869,618 MHz y ≤ 7 dBm.

Resultado esperado de `masn_osc_test.py 1v8`: `XOSC … mode=STDBY_XOSC errors=OK` y `FS … mode=FS errors=OK`. El `XOSC_START` que aparece en `POR`/`END` es el de encendido, antes de configurar el TCXO, y es normal.

La suite de diagnóstico del firmware (`MASN_Diagnostics.h`) responde a `CMD_DETECT` (`0x08`) con subcomandos `0x60`–`0x6F`:

| Código | Qué hace |
|---|---|
| `0x60` | Configuración |
| `0x61` / `0x62` | Estado del SX1262, IRQ, errores y contadores; `0x62` da también los contadores del vigilante de radio (`RECOV`, `RECOV_PT`, `RECOV_AGO`) y el tiempo encendido (`UP`, en segundos) |
| `0x63` / `0x64` | CW de 600 ms (`0x64` con RXEN en LOW) |
| `0x65` | Paquete LoRa de prueba |
| `0x66` | Borra errores e IRQ |
| `0x67` | Registros RF |
| `0x68`–`0x6A` | Traza del oscilador (TCXO 1,8 V / 3,3 V / cristal) |
| `0x6B` | Detecta qué GPIO sigue a BUSY |
| `0x6C` | Tensiones de alimentación, escaneo I2C y canales del INA3221 en bruto |
| `0x6D` | Estado de la copia de seguridad de la identidad y la configuración |
| `0x6E` | **Formatea el sistema de ficheros y reinicia**, para probar la restauración desde la copia |
| `0x6F` | Reinicia el SX1262 por su pin de reset, para probar el vigilante de radio |

## Problemas resueltos durante la adaptación

Por si te encuentras los mismos síntomas en otra placa:

1. **BUSY en un pin equivocado (P0.20 en vez de P0.29).** Los comandos se enviaban con el SX1262 ocupado y el chip los descartaba en silencio. El TCXO "no arrancaba" y TX fallaba de forma intermitente.
2. **P0.13 sin activar.** El MOSFET de VCC conducía a medias: bastaba para recibir, pero al encender el PA el SX1262 se reiniciaba. Se activa en `sx126x::preInit()`. Ponerlo en `initVariant()` del variant no funciona, porque ese símbolo nunca se enlaza.
3. **TCXO:** el DIO3 tiene que configurarse antes de calibrar.
4. **Memoria:** el pool de RNS necesita 128 KB (`RNS_HEAP_POOL_BUFFER_SIZE=131072`). Con 64 KB, RNS arranca con `bad_alloc` y descarta todos los paquetes (`No identity!`).
5. **Bloqueo en microReticulum (problema conocido de la librería, no de la placa):** al servir una página NomadNet por un *Link*, los reintentos del *Resource* bloquean `Transport::jobs()`, y el watchdog reinicia la placa a los 60 s. La corrección se ha enviado al repositorio de microReticulum.

## Limitaciones conocidas

- **Las rutas se guardan en RAM.** Las rutas, los destinos conocidos y la lista de paquetes no se escriben en la flash (`RNS_PERSIST_*=0`), porque eso suponía una escritura casi por cada anuncio recibido. Tras un reinicio el nodo las vuelve a aprender de los anuncios. La tabla de rutas admite 25 destinos (`URTN_PATH_TABLE_MAX_RECS`).
- **Cortes de corriente y sistema de ficheros.** Un corte mientras LittleFS escribe puede dejarlo sin poder montarse; el núcleo de Adafruit lo formatea entonces, y eso borraba el aprovisionamiento, la configuración y la identidad del nodo. Ahora el firmware guarda una copia de esos ficheros en dos páginas de flash fuera del sistema de ficheros (`MASN_Backup.h`, se actualiza 90 s después de arrancar y cada 10 minutos si algo cambió) y la restaura al arrancar. Los cambios hechos en los 10 minutos anteriores a un corte pueden perderse.
- **Vigilante de radio.** Tras varios días a la intemperie un nodo apareció sordo: el SX1262 estaba en reposo y nada lo devolvía a recepción. Ahora el firmware revisa el chip cada 5 s y repite la configuración de la radio si la ha perdido (el tipo de paquete ya no es LoRa, es decir, el chip se reinició) o si sigue en reposo 15 s. `RECOV_PT` en el diagnóstico `0x62` indica qué caso fue (0 = reinicio del chip, 1 = se quedó en reposo). La causa de fondo (probablemente una bajada de tensión en la radio) no está confirmada.
- **Los cambios de aprovisionamiento pueden perderse al reiniciar (de forma intermitente).** Se ha visto que un commit de aprovisionamiento seguido poco después de guardar la configuración de radio y reiniciar vuelve con los valores anteriores. `masn_setup.py` graba primero la radio, hace el commit al final y, tras el reinicio, relee y vuelve a grabar la configuración hasta que se conserva. Si configuras a mano, compruébalo con `masn_setup.py show` después de reiniciar.
- **Un ordenador que deja el puerto USB abierto sin leerlo reinicia el nodo.** El firmware escribe su log por USB; cuando el búfer se llena se bloquea, y el watchdog de 60 s reinicia la placa. Lee o cierra el puerto; `masn_setup.py` hace ambas cosas. A un nodo sin nada conectado por USB no le afecta.
- El aprovisionamiento de una placa nueva, la MASN V2.0 y la variante Core1262 no están verificados.

## Pendiente

- Leer el sensor ambiental (BME280) y publicar la batería y el ambiente como telemetría.
- Probar la MASN V2.0 y la variante con Core1262.
- Añadir el modelo `0x53` a `rnodeconf`, para poder usar `--autoinstall`.
- Medir el consumo en modo nodo y ajustar la gestión de energía.
