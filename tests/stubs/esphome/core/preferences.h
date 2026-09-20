#pragma once
namespace esphome {
struct ESPPreferenceObject {template<class T> bool load(T*){return false;} template<class T> bool save(const T*){return true;}};
struct Preferences {template<class T> ESPPreferenceObject make_preference(unsigned,bool=true){return {};}};
inline Preferences *global_preferences=nullptr;
}
