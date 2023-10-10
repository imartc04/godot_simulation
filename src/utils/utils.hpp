

#pragma once

#include <string>
#include <godot_cpp/variant/string.hpp>
#include <map>

namespace godot
{

    // Convert a godot::String to std::string
    static std::string godotStringToStdString(const String &godotString)
    {
        return godotString.utf8().get_data();
    }

}

template <typename K, typename V>
static std::map<V, K> reverse_map(const std::map<K, V> &m)
{
    std::map<V, K> r;
    for (const auto &kv : m)
        r[kv.second] = kv.first;
    return r;
}