// mjc_net_interface.mm
// Network interface detection — enumerates en* (WiFi/Ethernet) IPv4 addresses.

#include "mjc_net_interface.h"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <cstring>

#import <Foundation/Foundation.h>

namespace imujoco {

std::vector<std::string> GetLocalAddresses() {
    std::vector<std::string> addresses;
    struct ifaddrs* ifaddr = nullptr;

    if (getifaddrs(&ifaddr) != 0) return addresses;

    for (auto* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) continue;
        if (ifa->ifa_addr->sa_family != AF_INET) continue;

        std::string name(ifa->ifa_name);
        if (name.compare(0, 2, "en") != 0) continue;  // Only en* interfaces

        char buf[INET_ADDRSTRLEN];
        auto* sa = reinterpret_cast<struct sockaddr_in*>(ifa->ifa_addr);
        if (inet_ntop(AF_INET, &sa->sin_addr, buf, sizeof(buf))) {
            addresses.emplace_back(buf);
        }
    }

    freeifaddrs(ifaddr);
    return addresses;
}

std::string GetLocalBindAddress() {
    @autoreleasepool {
        // Read user's preferred address from UserDefaults
        NSString* preferred = [[NSUserDefaults standardUserDefaults] stringForKey:@"bindAddress"];
        std::string pref;
        if (preferred.length > 0) {
            pref = std::string([preferred UTF8String]);
        }

        auto addresses = GetLocalAddresses();

        // If preferred matches an available en* address, use it
        if (!pref.empty()) {
            for (const auto& addr : addresses) {
                if (addr == pref) return pref;
            }
        }

        // Auto-select first en* address
        if (!addresses.empty()) return addresses[0];

        // Fallback to localhost
        return "127.0.0.1";
    }
}

}  // namespace imujoco
