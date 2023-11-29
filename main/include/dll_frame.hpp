#ifndef DLL_FRAME_HPP
#define DLL_FRAME_HPP

#include <iostream>

namespace cadmium::comms {
    struct dll_frame {
        uint32_t data;
        uint8_t frame_num; //5 bits -> 32 frames
        uint8_t total_frames;
        uint8_t checksum;

        explicit dll_frame(): data(0), frame_num(0), total_frames(0), checksum(0) {};
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
        out << "0x" << std::hex << frame.data;
        out << ", checksum: ";
        out << "0x" << std::hex << (unsigned int)frame.checksum;
        out << ", frame_num: ";
        out << "0x" << std::hex << (unsigned int)frame.frame_num;
        out << ", total_frames: ";
        out << "0x" << std::hex << (unsigned int)frame.total_frames << " }";
		
		return out;
	}
#endif
}

#endif