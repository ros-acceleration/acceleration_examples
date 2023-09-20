#include "avalonmm_driver.hpp"
#include <iostream>

uint64_t AvalonMM_Driver::read_register(uint32_t offset) {
    ensure_mmio_mapped();
    return mmio_region[offset / sizeof(uint64_t)];
}

void AvalonMM_Driver::write_register(uint32_t offset, uint64_t value) {
    ensure_mmio_mapped();
    mmio_region[offset / sizeof(uint64_t)] = value;
}

void AvalonMM_Driver::ensure_mmio_mapped() {
    if (!mmio_region) {
        fd = open("/dev/mem", O_RDWR | O_SYNC);
        if (fd == -1) {
            // TODO: Add error handling, e.g., throw an exception.
        }

        mmio_region = (volatile uint64_t *)mmap(nullptr, MMIO_REGION_SIZE, 
            PROT_READ | PROT_WRITE, MAP_SHARED, fd, MMIO_BASE_ADDRESS);
        
        if (mmio_region == MAP_FAILED) {
            close(fd);
            fd = -1;
            // TODO: Add error handling, e.g., throw an exception.
        }
    }
}

void AvalonMM_Driver::cleanup_mmio() {
    if (mmio_region) {
        munmap((void *)mmio_region, MMIO_REGION_SIZE);
        mmio_region = nullptr;
    }

    if (fd != -1) {
        close(fd);
        fd = -1;
    }
}

AvalonMM_Driver::AvalonMM_Driver(uintptr_t baseAddress, size_t regionSize)
    : MMIO_BASE_ADDRESS(baseAddress), MMIO_REGION_SIZE(regionSize), fd(-1), mmio_region(nullptr) {}

void AvalonMM_Driver::set_arg(uintptr_t register_addr, uint32_t memory_offset, int* values, size_t length) {
    ensure_mmio_mapped();
    write_register(register_addr, memory_offset);  // map register address to memory offset
    uint32_t offset = memory_offset;
    for (size_t i = 0; i < length; ++i, offset += sizeof(uint64_t)) {
        write_register(offset, values[i]);
    }
}

void AvalonMM_Driver::get_arg(uint32_t memory_offset, int* values, size_t length){
    ensure_mmio_mapped();
    uint32_t offset = memory_offset;
    for (size_t i = 0; i < length; ++i, offset += sizeof(uint64_t)) {
        values[i] = read_register(offset);
    }
}

void AvalonMM_Driver::start_kernel(uint32_t status_reg, uint64_t go_mask) {
    uint64_t status = read_register(status_reg);
    status |= go_mask;
    write_register(status_reg, status);
}

bool AvalonMM_Driver::is_kernel_done(uint32_t status_reg, uint64_t done_mask) {
    uint64_t status = read_register(status_reg);
    return (status & done_mask) != 0;
}

int AvalonMM_Driver::get_value(uint32_t offset) {
    return read_register(offset);
}

void AvalonMM_Driver::set_value(uint32_t offset, int value) {
    write_register(offset, value);
}

// Dumps the contents of arg_c_out to stdout based on length.
void AvalonMM_Driver::dump_arg_values(uint32_t offset, size_t length) {
    std::cout << "args (@ 0x" << std::hex << offset << " ):  ";
    for (size_t i = 0; i < length; ++i) {
        uint64_t c_out_val = read_register(offset + i * sizeof(uint64_t));
        std::cout << "0x" << std::hex << c_out_val << " ";
    }
    std::cout << std::endl;
}


AvalonMM_Driver::~AvalonMM_Driver() {
    cleanup_mmio();
}
