// args.h
// Lightweight --key value argument parser for C++ examples.
// Header-only, no external dependencies.

#pragma once

#include <map>
#include <string>
#include <cstdlib>

namespace imujoco::driver {

class Args {
public:
    Args(int argc, char* argv[]) {
        if (argc > 0) {
            program_ = argv[0];
        }
        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            if (arg.size() > 2 && arg[0] == '-' && arg[1] == '-') {
                std::string key = arg.substr(2);
                if (i + 1 < argc && !(argv[i + 1][0] == '-' && argv[i + 1][1] == '-')) {
                    args_[key] = argv[++i];
                } else {
                    args_[key] = "";
                }
            }
        }
    }

    std::string get(const std::string& key, const std::string& def) const {
        auto it = args_.find(key);
        return it != args_.end() ? it->second : def;
    }

    int get_int(const std::string& key, int def) const {
        auto it = args_.find(key);
        if (it == args_.end() || it->second.empty()) return def;
        return std::atoi(it->second.c_str());
    }

    double get_double(const std::string& key, double def) const {
        auto it = args_.find(key);
        if (it == args_.end() || it->second.empty()) return def;
        return std::atof(it->second.c_str());
    }

    bool has(const std::string& key) const {
        return args_.find(key) != args_.end();
    }

    const std::string& program() const { return program_; }

private:
    std::map<std::string, std::string> args_;
    std::string program_;
};

}  // namespace imujoco::driver
