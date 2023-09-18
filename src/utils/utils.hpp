

#pragma once

#include <string>
#include <godot_cpp/variant/string.hpp>


namespace godot
{

    // Convert a godot::String to std::string
    static std::string godotStringToStdString(const String &godotString)
    {
        PoolStringArray array = godotString.utf8();
        if (array.size() > 0)
        {
            const char *utf8Chars = array[0].c_str();
            return std::string(utf8Chars);
        }
        return std::string(); // Return an empty string if conversion fails
    }

}