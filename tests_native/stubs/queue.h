#pragma once
#include <deque>
#include <vector>
#include <cstring>
struct Queue { size_t size; std::deque<std::vector<uint8_t>> items; };
using QueueHandle_t=Queue*;
inline int xQueueSend(Queue* q,const void* data,int){
 const uint8_t* p=(const uint8_t*)data; q->items.emplace_back(p,p+q->size); return 1;
}
inline int xQueueReceive(Queue* q,void* data,int){
 if(q->items.empty())return 0;
 memcpy(data,q->items.front().data(),q->size); q->items.pop_front(); return 1;
}
