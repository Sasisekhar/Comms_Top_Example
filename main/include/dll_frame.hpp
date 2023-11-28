#ifndef DLL_FRAME_HPP
#define DLL_FRAME_HPP

#include <iostream>

namespace cadmium{
    struct dll_frame {
        uint32_t data;
        uint8_t frame_num; //6 bits -> 64 frames
        uint8_t checksum;

        explicit dll_frame(): data(0), frame_num(0), checksum(0) {};
    };

#ifndef NO_LOGGING
	/**
	 * Insertion operator for dll_frame objects.
	 * @param out output stream.
	 * @param b bid to be represented in the output stream.
	 * @return output stream with the value of the bid already inserted.
	 */
	std::ostream& operator<<(std::ostream& out, const dll_frame& frame) {
		out << "{ data: ";
        out << std::hex << frame.data;
        out << ", checksum: ";
        out << std::hex << frame.checksum;
        out << ", framenum: " << frame.frame_num << " }";
		
		return out;
	}
#endif
}

#endif