#pragma once
#include <cstring>
#include <map>
#include <vector>
namespace esphome {
inline std::map<unsigned, std::vector<unsigned char>> mock_preferences;
struct ESPPreferenceObject {
 unsigned key{0};
 template<class T> bool load(T *value) {
   const auto it=mock_preferences.find(key);
   if(it==mock_preferences.end() || it->second.size()!=sizeof(T)) return false;
   std::memcpy(value,it->second.data(),sizeof(T));return true;
 }
 template<class T> bool save(const T *value) {
   const auto *bytes=reinterpret_cast<const unsigned char *>(value);
   mock_preferences[key]={bytes,bytes+sizeof(T)};return true;
 }
};
struct Preferences {template<class T> ESPPreferenceObject make_preference(unsigned key,bool=true){return {key};}};
inline Preferences *global_preferences=nullptr;
}
