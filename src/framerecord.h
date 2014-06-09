
#ifndef SLOW_FRAMERECORD_H_
#define SLOW_FRAMERECORD_H_

#include <iostream>

class FrameRecord {
public:
    FrameRecord();
    ~FrameRecord();
    FrameRecord(const FrameRecord&) = delete;            // no copy
    FrameRecord& operator=(const FrameRecord&) = delete; // no assign
    
};

#endif /* defined(SLOW_FRAMERECORD_H_) */
