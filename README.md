# lora_mesh
Implementing LoRa Mesh on Adafruit M0 Feather Boards using the Radiohead library and Arduino. For more information, consult [Adafruit Arduino IDE Setup](https://learn.adafruit.com/adafruit-feather-m0-basic-proto/setup) and [RadioHead Docs](http://www.airspayce.com/mikem/arduino/RadioHead/index.html).

Node 1 is setup to respond to any messages that it gets, while all other nodes are setup to send messages to node 1 (acts as a bridge node). To add more nodes to the network, modify the address and message of the node 2 file. The code for node 2 can then be deployed on up to 253 additional nodes.

Code adapted from [Nootropic](https://github.com/nootropicdesign/lora-mesh)  and [Adafruit](https://github.com/adafruit/RadioHead/tree/master/examples/feather).