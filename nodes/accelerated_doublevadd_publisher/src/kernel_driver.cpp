// Defines the offset for the register map and the base address of the MMIO region.
#define ZTS11VECTORADDID_REGISTER_MAP_OFFSET (0x80000)
#define KERNEL_DRIVER_MMMIO_BASE_ADDRESS (0xF9000000)

#include "ZTS11VectorAddID_register_map.hpp"
#include <cstdint>
#include <iostream>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

enum MemoryOffsets {
    ARG_A_OFFSET = 0x000,
    ARG_B_OFFSET = 0x100,
    ARG_C_OFFSET = 0x300,
    ARG_LEN_OFFSET = 0x200
};

// Driver class for the ZTS11VectorAddID device.
class ZTS11VectorAddID_Driver {
private:
    // Base address of the MMIO region for the device.
    static const uintptr_t MMIO_BASE_ADDRESS = KERNEL_DRIVER_MMMIO_BASE_ADDRESS;
    
    // Size of the MMIO region that includes memory addresses to the kernel and registers.
    static const size_t MMIO_REGION_SIZE = 0x90000; 

    int fd;  // File descriptor for accessing memory through /dev/mem.
    volatile uint64_t *mmio_region; // Pointer to the mapped MMIO region.

    // Reads a value from a register at a given offset.
    uint64_t read_register(uint32_t offset) {
        ensure_mmio_mapped();
        return mmio_region[offset / sizeof(uint64_t)];
    }

    // Writes a value to a register at a given offset.
    void write_register(uint32_t offset, uint64_t value) {
        ensure_mmio_mapped();
        mmio_region[offset / sizeof(uint64_t)] = value;
    }

    // General function to set kernel arguments.
    void set_arg_values(uint32_t base_offset, uint64_t* values, size_t length) {
        ensure_mmio_mapped();
        uint32_t offset = base_offset;
        for (size_t i = 0; i < length; ++i, offset += sizeof(uint64_t)) {
            write_register(offset, values[i]);
        }
    }

    // Ensures the MMIO region is mapped. If not, it attempts to map it.
    void ensure_mmio_mapped() {
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

    // Releases resources associated with the MMIO region.
    void cleanup_mmio() {
        if (mmio_region) {
            munmap((void *)mmio_region, MMIO_REGION_SIZE);
            mmio_region = nullptr;
        }

        if (fd != -1) {
            close(fd);
            fd = -1;
        }
    }

public:
    // Constructor initializes the file descriptor and MMIO region pointer.
    ZTS11VectorAddID_Driver() : fd(-1), mmio_region(nullptr) {}

    // Initiates kernel execution.
    void start_kernel() {
        uint64_t status = read_register(ZTS11VECTORADDID_REGISTER_MAP_STATUS_REG);
        status |= KERNEL_REGISTER_MAP_GO_MASK; // Set the GO bit.
        write_register(ZTS11VECTORADDID_REGISTER_MAP_STATUS_REG, status);
    }

    // Checks if the kernel has finished execution.
    bool is_kernel_done() {
        uint64_t status = read_register(ZTS11VECTORADDID_REGISTER_MAP_STATUS_REG);
        return (status & KERNEL_REGISTER_MAP_DONE_MASK) != 0; // Check the DONE bit.
    }

    // Methods for setting kernel arguments using arrays:
    void set_arg_a_in(uint64_t* values, size_t length) {
        set_arg_values(ARG_A_OFFSET, values, length);
        write_register(ZTS11VECTORADDID_REGISTER_MAP_ARG_ARG_A_IN_REG, ARG_A_OFFSET);
    }    

    void set_arg_b_in(uint64_t* values, size_t length) {
        set_arg_values(ARG_B_OFFSET, values, length);
        write_register(ZTS11VECTORADDID_REGISTER_MAP_ARG_ARG_B_IN_REG, ARG_B_OFFSET);
    }

    void set_arg_c_out(uint64_t* values, size_t length) {
        set_arg_values(ARG_C_OFFSET, values, length);
        write_register(ZTS11VECTORADDID_REGISTER_MAP_ARG_ARG_C_OUT_REG, ARG_C_OFFSET);
    }

    void set_arg_len(uint32_t value) {
        write_register(ZTS11VECTORADDID_REGISTER_MAP_ARG_ARG_LEN_REG, ARG_LEN_OFFSET);
        write_register(ARG_LEN_OFFSET, value);
    }

    // Methods for fetching results or status:
    uint64_t get_finish_counter() {
        return read_register(ZTS11VECTORADDID_REGISTER_MAP_FINISHCOUNTER_REG1);
    }

    // Dumps the contents of arg_c_out to stdout based on length.
    void dump_arg_c_out(size_t length) {
        std::cout << "arg_c_out values: ";
        for (size_t i = 0; i < length; ++i) {
            uint64_t c_out_val = read_register(0x300 + i * sizeof(uint64_t));
            std::cout << std::hex << c_out_val << " ";
        }
        std::cout << std::endl;
    }

    ~ZTS11VectorAddID_Driver() {
        cleanup_mmio();
    }    
};

int main() {
    ZTS11VectorAddID_Driver driver;

    uint64_t arg_a_values[] = {0x62, 0x63, 0x64, 0x65, 0x66, 0x67};
    uint64_t arg_b_values[] = {0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    uint64_t arg_c_values[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    size_t length = sizeof(arg_a_values) / sizeof(uint64_t);

    // Set kernel arguments and initiate kernel execution.
    driver.set_arg_a_in(arg_a_values, length);
    driver.set_arg_b_in(arg_b_values, length);
    driver.set_arg_c_out(arg_c_values, length);
    driver.set_arg_len(length);
    driver.start_kernel();

    // Wait for the kernel to finish execution.
    while (!driver.is_kernel_done()) {}

    // Display results.
    driver.dump_arg_c_out(length);

    // uint64_t finish_counter = driver.get_finish_counter();
    // std::cout << "Kernel finish counter: " << finish_counter << std::endl;
    return 0;
}
