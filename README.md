# Paradox MG6250 serial to MQTT (mg6250_mqtt)

This project aims to process serial messages sent by the MG6250 Alarm console and report basic alarm events to an MQTT broker.

The MG6250 console serial protocol was reversed engineered to some extent. Currently, it is only possible to receive messages from the console, not writing to the serial port.

<div style="text-align:center">
  <img src="/assets/project1.jpg" alt="mg6250_ESP8266" width="40%" />
</div>

## Serial port pin layout

The pin layout for the MG6250 is not the same as older consoles such as the MG5050. The Tx and Rx pins are inverted.

     MG6250
    ┌───────┐
    │ Tx   ┌╵
    │ Rx   │
    │ GND  │
    │ AUX+ └╷
    └───────┘

## Message Protocol

Messages have a variable length depending on the number of events reported. The message length is sent on the 4th byte of the message.

Each message has the following format:

    |_ _ _ C _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ (...)|_ _|
           7                        37 * E                      2
        ctr bytes                 event bytes             checksum bytes

    where C is the message lengh and E the number of events.

Most of the bytes are still unknown. However, the known bytes are enough for identifying the desired events (e.g. event type and sub-type).

Here is the structure for each Event in the message:

{% highlight cpp %}
struct Event
{
  byte seq;
  byte unknown8;
  byte unknown9;
  byte unknown10;
  byte unknown11;
  byte unknown12;
  byte event;
  byte sub_event;
  byte area;
  byte century;
  byte year;
  byte month;
  byte day;
  byte hour;
  byte minute;
  byte unknown13;
  byte unknown14;
  byte unknown15;
  byte unknown16;
  byte unknown17;
  byte typed;
  char label[17];
};
{% endhighlight %}


### Event decoding

The MG6250 Section Programming Guide contains the event and sub-event codes am their meaning.

![MG6250 Section Programming Guide](/assets/mg6250_guide2.png)


### Checksum

After much trial and error, I also managed to figure out the CRC algorithm used for the checksum. The protocol uses the **CRC-16/MODBUS.** The high order byte is the 46th byte.

## Source code

This repository contains two files:
 * [paradox_serial_connect.py](/paradox_serial_connect.py)
 * [ParadoxMG6250.ino](/ParadoxMG6250.ino)

The ```paradox_serial_connect.py``` is a python program used to test the protocol. Using this script, the MG6250 can be connected to a PC direclty without a microcontroller.

The ```ParadoxMG6250.ino``` source file is a C++ sketch designed to work with an ESP8266 microcontroller.

## Whats next?

 * Communicate with the alarm console in order to Arm or Disarm the alarm remotely.
 * Improve battery performance. Right now, the sketch does not support any level of deep sleep, resulting in the worst performance as the board is always running.

## Reference work

  * PAI (Paradox Alarm Interface): [https://github.com/ParadoxAlarmInterface/pai](https://github.com/ParadoxAlarmInterface/pai)
  * paradox_mqtt: [https://github.com/spinza/paradox_mqtt](https://github.com/spinza/paradox_mqtt)
  * [https://harizanov.com/2014/07/interfacing-with-paradox-home-security-system-attempt-2/](https://harizanov.com/2014/07/interfacing-with-paradox-home-security-system-attempt-2/)
