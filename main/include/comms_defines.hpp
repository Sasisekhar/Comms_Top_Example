#ifndef COMMS_DEFINES_HPP
#define COMMS_DEFINES_HPP

namespace cadmium::comms{
    #define PAYLOAD_LEN     32
    #define FRAME_NUM_LEN   5
    #define SELECT_LEN      1
    #define CHECKSUM_LEN    8

    // Masks for extracting fields from a 64-bit frame
    #define PAYLOAD_MASK        ((1ULL << PAYLOAD_LEN) - 1)
    #define FRAME_MASK          (((1ULL << FRAME_NUM_LEN) - 1) << PAYLOAD_LEN)
    #define TOTAL_FRAMES_MASK   (((1ULL << FRAME_NUM_LEN) - 1) << (PAYLOAD_LEN + FRAME_NUM_LEN))
    #define SELECT_MASK         (((1ULL << SELECT_LEN) - 1) << (PAYLOAD_LEN + FRAME_NUM_LEN + FRAME_NUM_LEN))
    #define CHECKSUM_MASK       (((1ULL << CHECKSUM_LEN) - 1) << (PAYLOAD_LEN + FRAME_NUM_LEN + FRAME_NUM_LEN + SELECT_LEN))
}

#endif