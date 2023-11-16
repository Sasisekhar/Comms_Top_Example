#ifndef TRANSMISSION_HPP_
#define TRANSMISSION_HPP_

#include <iostream>

namespace cadmium {
	struct dataframe {
		std::vector<uint32_t> data;
        // uint32_t data[3];
		
		/**
		 * Constructor function for a Bid object.
		 * @param id  ID of the new bid/ask.
		 * @param InitialPPr As the first bid comes as an input, we generate the corresponding parameter. !!!I don't think this is done here
		 * @ref Purpr is initialized as the initial purchase price.
		 */
		explicit dataframe(): data(0) {};
	};

	/**
	 * Insertion operator for Bid objects.
	 * @param out output stream.
	 * @param b bid to be represented in the output stream.
	 * @return output stream with the value of the bid already inserted.
	 */
	std::ostream& operator<<(std::ostream& out, const dataframe& b) {
		out << "{";
        for(auto x : b.data){
            out << std::hex << x << ", ";
        }
        out << "}";
		
		return out;
	}
}  //namespace cadmium

#endif //TRANSMISSION_HPP_