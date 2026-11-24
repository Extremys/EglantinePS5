// SPDX-License-Identifier: GPL-3.0-or-later
// MihawkGen's trained network (models/network.fgw, 1.7 MB), carried in the program: the assembler
// takes the file as it is (FG_NETWORK_PATH, set by the build), so no generated source holds it.
#include <cstddef>

#ifndef FG_NETWORK_PATH
#error "FG_NETWORK_PATH (the absolute path of models/network.fgw) must be defined"
#endif

__asm__(".section .rodata\n"
        ".balign 16\n"
        ".globl fg_network_blob_start\n"
        "fg_network_blob_start:\n"
        ".incbin \"" FG_NETWORK_PATH "\"\n"
        ".globl fg_network_blob_end\n"
        "fg_network_blob_end:\n"
        ".previous\n");

extern "C" const unsigned char fg_network_blob_start[];
extern "C" const unsigned char fg_network_blob_end[];

extern "C" const unsigned char* const fg_network_blob = fg_network_blob_start;
extern "C" const std::size_t fg_network_blob_size =
    static_cast<std::size_t>(fg_network_blob_end - fg_network_blob_start);
