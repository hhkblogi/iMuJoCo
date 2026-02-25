// mjc_net_interface.h
// Network interface detection utility.
//
// Restricts server socket binding to WiFi/Ethernet (en*) interfaces + localhost.
// Reads the user's preferred bind address from NSUserDefaults["bindAddress"].

#ifndef mjc_net_interface_h
#define mjc_net_interface_h

#include <string>
#include <vector>

namespace imujoco {

/// Get the best local bind address for server sockets.
/// Reads "bindAddress" from NSUserDefaults. If that value matches an available
/// en* interface, returns it. Otherwise auto-selects the first en* interface
/// IPv4 address. Falls back to "127.0.0.1" if no en* interfaces are found.
std::string GetLocalBindAddress();

/// Returns all IPv4 addresses on en* interfaces (WiFi/Ethernet).
/// Used by the UI to populate the IP address picker.
std::vector<std::string> GetLocalAddresses();

}  // namespace imujoco

#endif /* mjc_net_interface_h */
