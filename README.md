# lora_mesh
Implementing LoRa Mesh on Adafruit M0 Feather Boards using the Radiohead library and Arduino. For more information, consult [Adafruit Arduino IDE Setup](https://learn.adafruit.com/adafruit-feather-m0-basic-proto/setup) and [RadioHead Docs](http://www.airspayce.com/mikem/arduino/RadioHead/index.html).

To deploy code onto nodes, download the repository and use the respective ino files provided in the lora_mesh folder.

Node 1 is setup to respond to any messages that it gets, while all other nodes are setup to send messages to node 1 (acts as a bridge node). To add more nodes to the network, modify the address and message of the node 2 file. The code for node 2 can then be deployed on up to 253 additional nodes.

Code adapted from [Nootropic](https://github.com/nootropicdesign/lora-mesh)  and [Adafruit](https://github.com/adafruit/RadioHead/tree/master/examples/feather).

To configure LoRa parameters for a node, open a terminal and navigate to the repository's root directory. From there, run either ./run (Windows) or ./run.sh (Mac/Linux). The web interface for configuration will open and can be accessed in a web browser at localhost:6543.
