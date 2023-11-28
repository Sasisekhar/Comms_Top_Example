#ifndef TCL_PACKET_HPP_
#define TCL_PACKET_HPP_

#include <iostream>

namespace cadmium {
	struct tcl_packet {
		std::vector<uint32_t> data;
		uint8_t atomic_id;
		
		/**
		 * Constructor function for a Bid object.
		 * @param id  ID of the new bid/ask.
		 * @param InitialPPr As the first bid comes as an input, we generate the corresponding parameter. !!!I don't think this is done here
		 * @ref Purpr is initialized as the initial purchase price.
		 */
		explicit tcl_packet(): data(0), atomic_id(0) {};
	};

#ifndef NO_LOGGING
	/**
	 * Insertion operator for Bid objects.
	 * @param out output stream.
	 * @param b bid to be represented in the output stream.
	 * @return output stream with the value of the bid already inserted.
	 */
	std::ostream& operator<<(std::ostream& out, const tcl_packet& b) {
		out << "{";
        for(auto x : b.data){
            out << std::hex << x << ", ";
        }
        out << "}";
		
		return out;
	}
#endif

}  //namespace cadmium

#endif //TRANSMISSION_HPP_