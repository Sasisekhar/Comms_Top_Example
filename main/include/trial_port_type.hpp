#ifndef TRIAL_PORT_TYPE_HPP
#define TRIAL_PORT_TYPE_HPP

#include <iostream>

namespace cadmium::example {
    struct RGB_val {
        // uint16_t red;
        // bool green;
        uint8_t blue[5];

        //explicit RGB_val(): /*red(0), green(false), blue(0)*/{};
    };

#ifndef NO_LOGGING
	/**
	 * Insertion operator for dll_frame objects.
	 * @param out output stream.
	 * @param b bid to be represented in the output stream.
	 * @return output stream with the value of the bid already inserted.
	 */
	std::ostream& operator<<(std::ostream& out, const RGB_val& clr) {
		out << "{ red: ";
        // out << "0x" << std::hex << (unsigned int)clr.red;
        // out << ", green: ";
        // out << "0x" << std::hex << (unsigned int)clr.green;
        // out << ", blue: ";
        out << "0x" << std::hex << (unsigned int)clr.blue[0];
        out << " }";
		
		return out;
	}
#endif
}

#endif