#ifndef INTERRUPT_HANDLER_HPP
#define INTERRUPT_HANDLER_HPP

namespace cadmium {
    
    template<typename  decodeType>
    class InterruptHandler {
        
        public:

        virtual bool ISRcb() = 0;

        
        virtual decodeType decodeISR() = 0;

    };
}

#endif