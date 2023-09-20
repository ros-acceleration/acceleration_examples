#ifndef AVALONMM_DRIVER_HPP
#define AVALONMM_DRIVER_HPP

#include <cstdint>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

class AvalonMM_Driver {
private:
    uintptr_t MMIO_BASE_ADDRESS;
    size_t MMIO_REGION_SIZE;
    int fd;
    volatile uint64_t *mmio_region;

    void write_register(uint32_t offset, uint64_t value);
    void ensure_mmio_mapped();
    void cleanup_mmio();

public:
    AvalonMM_Driver(uintptr_t baseAddress, size_t regionSize);

    uint64_t read_register(uint32_t offset);
    void set_arg(uintptr_t register_addr, uint32_t memory_offset, int* values, size_t length);
    void get_arg(uint32_t memory_offset, int* values, size_t length);
    void start_kernel(uint32_t status_reg, uint64_t go_mask);
    bool is_kernel_done(uint32_t status_reg, uint64_t done_mask);
    int get_value(uint32_t offset);
    void set_value(uint32_t offset, int value);
    void dump_arg_values(uint32_t offset, size_t length);
    ~AvalonMM_Driver();
};

#endif // AVALONMM_DRIVER_HPP
