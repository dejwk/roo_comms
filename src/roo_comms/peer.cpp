#include "roo_comms/peer.h"

namespace roo_comms {

Peer::Peer(const roo_io::MacAddress& addr) : EspNowPeer(Transport(), addr) {}

}