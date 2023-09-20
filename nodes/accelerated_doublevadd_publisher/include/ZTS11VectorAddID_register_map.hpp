
/* This header file describes the Register Map for the ZTS11VectorAddID kernel */

#ifndef __ZTS11VECTORADDID_REGISTER_MAP_REGS_H__
#define __ZTS11VECTORADDID_REGISTER_MAP_REGS_H__



/* Status register contains all the control bits to control kernel execution */
/******************************************************************************/
/* Memory Map Summary                                                         */
/******************************************************************************/

/*
  Address   | Access  | Register              | Argument
------------|---------|-----------------------|-----------------------------
        0x0 |     R/W |       register0[31:0] |                Status[31:0]
------------|---------|-----------------------|-----------------------------
       0x28 |     R/W |       register5[31:0] |         FinishCounter[31:0]
            |         |      register5[63:32] |         FinishCounter[31:0]
------------|---------|-----------------------|-----------------------------
       0x78 |     R/W |      register15[63:0] |              arg_a_in[63:0]
------------|---------|-----------------------|-----------------------------
       0x80 |     R/W |      register16[63:0] |              arg_b_in[63:0]
------------|---------|-----------------------|-----------------------------
       0x88 |     R/W |      register17[63:0] |             arg_c_out[63:0]
------------|---------|-----------------------|-----------------------------
       0x90 |     R/W |      register18[31:0] |               arg_len[31:0]
*/


/******************************************************************************/
/* Register Address Macros                                                    */
/******************************************************************************/

/* Status Register Bit Offsets (Bits) */
/* Note: Bits In Status Registers Are Marked As Read-Only or Read-Write
   Please Do Not Write To Read-Only Bits */
#define KERNEL_REGISTER_MAP_GO_OFFSET (0) // Read-write
#define KERNEL_REGISTER_MAP_DONE_OFFSET (1) // Read-only
#define KERNEL_REGISTER_MAP_BUSY_OFFSET (2) // Read-only
#define KERNEL_REGISTER_MAP_STALLED_OFFSET (3) // Read-only
#define KERNEL_REGISTER_MAP_UNSTALL_OFFSET (4) // Read-write
#define KERNEL_REGISTER_MAP_VALID_IN_OFFSET (14) // Read-only
#define KERNEL_REGISTER_MAP_STARTED_OFFSET (15) // Read-only

/* Status Register Bit Masks (Bits) */
#define KERNEL_REGISTER_MAP_GO_MASK (0x1)
#define KERNEL_REGISTER_MAP_DONE_MASK (0x2)
#define KERNEL_REGISTER_MAP_BUSY_MASK (0x4)
#define KERNEL_REGISTER_MAP_STALLED_MASK (0x8)
#define KERNEL_REGISTER_MAP_UNSTALL_MASK (0x10)
#define KERNEL_REGISTER_MAP_VALID_IN_MASK (0x4000)
#define KERNEL_REGISTER_MAP_STARTED_MASK (0x8000)

/* Byte Addresses */
#define ZTS11VECTORADDID_REGISTER_MAP_STATUS_REG (0x0 + ZTS11VECTORADDID_REGISTER_MAP_OFFSET)
#define ZTS11VECTORADDID_REGISTER_MAP_FINISHCOUNTER_REG1 (0x28 + ZTS11VECTORADDID_REGISTER_MAP_OFFSET)
#define ZTS11VECTORADDID_REGISTER_MAP_FINISHCOUNTER_REG2 (0x2C + ZTS11VECTORADDID_REGISTER_MAP_OFFSET)
#define ZTS11VECTORADDID_REGISTER_MAP_ARG_ARG_A_IN_REG (0x78 + ZTS11VECTORADDID_REGISTER_MAP_OFFSET)
#define ZTS11VECTORADDID_REGISTER_MAP_ARG_ARG_B_IN_REG (0x80 + ZTS11VECTORADDID_REGISTER_MAP_OFFSET)
#define ZTS11VECTORADDID_REGISTER_MAP_ARG_ARG_C_OUT_REG (0x88 + ZTS11VECTORADDID_REGISTER_MAP_OFFSET)
#define ZTS11VECTORADDID_REGISTER_MAP_ARG_ARG_LEN_REG (0x90 + ZTS11VECTORADDID_REGISTER_MAP_OFFSET)

/* Argument Sizes (bytes) */
#define ZTS11VECTORADDID_REGISTER_MAP_ARG_ARG_A_IN_SIZE (8)
#define ZTS11VECTORADDID_REGISTER_MAP_ARG_ARG_B_IN_SIZE (8)
#define ZTS11VECTORADDID_REGISTER_MAP_ARG_ARG_C_OUT_SIZE (8)
#define ZTS11VECTORADDID_REGISTER_MAP_ARG_ARG_LEN_SIZE (4)

/* Argument Masks */
#define ZTS11VECTORADDID_REGISTER_MAP_ARG_ARG_A_IN_MASK (0xffffffffffffffffULL)
#define ZTS11VECTORADDID_REGISTER_MAP_ARG_ARG_B_IN_MASK (0xffffffffffffffffULL)
#define ZTS11VECTORADDID_REGISTER_MAP_ARG_ARG_C_OUT_MASK (0xffffffffffffffffULL)
#define ZTS11VECTORADDID_REGISTER_MAP_ARG_ARG_LEN_MASK (0xffffffffULL)

#endif /* __ZTS11VECTORADDID_REGISTER_MAP_REGS_H__ */
