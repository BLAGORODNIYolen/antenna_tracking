#include <cstdint>
#include <cassert>
#include <iostream>
#include "../firmware/AntennaTracker/strict_json.h"
#include "../firmware/lib/ArduinoJson-6.21.5/src/ArduinoJson.h"
int main() {
  assert(strictFlatJson("{\"a\":1,\"b\":null,\"c\":-1.2e+3}"));
  for(const char* s:{"{}junk","{}{}","{\"a\":1,\"a\":2}","{\"a\":01}","{\"a\":1.}","{\"a\":NaN}","{\"a\":true,}","{\"a\":\"bad\\x\"}"}) assert(!strictFlatJson(s));
  StaticJsonDocument<768> d;
  deserializeJson(d,"{\"i\":2,\"f\":2.0,\"b\":true,\"huge\":4294967296,\"max\":4294967295}");
  assert(d["i"].is<int>()); assert(!d["f"].is<int>()); assert(!d["b"].is<uint32_t>());
  assert(!d["huge"].is<uint32_t>()); assert(d["max"].is<uint32_t>());
  std::cout<<"Native JSON validation passed\n";
}
