#ifndef RGB_HPP
#define RGB_HPP

#include <iostream>

namespace cadmium::comms::example {
    struct RGB {
        uint8_t R;
        uint8_t G;
        uint8_t B;

        bool operator==(const RGB& a) const {
            return (R == a.R && G == a.G && B == a.B);
        }

        RGB(): R(0), G(0), B(0) {};

        explicit RGB(uint8_t _r, uint8_t _g, uint8_t _b): R(_r), G(_g), B(_b) {};
    };

#ifndef NO_LOGGING
	/**
	 * Insertion operator for dll_frame objects.
	 * @param out output stream.
	 * @param b bid to be represented in the output stream.
	 * @return output stream with the value of the bid already inserted.
	 */
	std::ostream& operator<<(std::ostream& out, const RGB& colour) {
		out << "{ RGB: ";
        out << (unsigned int) colour.R << ", " << (unsigned int) colour.G << ", " << (unsigned int) colour.B;
        out << " }";
		
		return out;
	}
#endif
}

#endif