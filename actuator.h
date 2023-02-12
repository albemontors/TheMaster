#include "definitions.h"

class actuator{
    public:
        actuator(uint16_t canID);
        int update();
    private:
        uint16_t canID;
};