#pragma once

#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/watchdog.h"
#include "sm.pio.h"
#include <string.h>

#define EB_ADD_BITS 16
#define EB_BUFFER_SIZE 0x10000
#define EB_65C02_MAGIC_NUMBER 0x65C02

// set to 1 to enable snooping reads to 6502 peripherals
// NB: not possible with existing mux 
#define EB_CAN_SNOOP 0 

#define _EB_WRITE_FLAG 0b010
#define _EB_READ_FLAG 0b001
#define _EB_SNOOP_FLAG 0b100

enum eb_perm
{
    EB_PERM_NONE = 0,
    EB_PERM_READ_ONLY = _EB_READ_FLAG,
    EB_PERM_WRITE_ONLY = _EB_WRITE_FLAG,
    EB_PERM_READ_WRITE = (_EB_WRITE_FLAG | _EB_READ_FLAG),
#if EB_CAN_SNOOP==1
    EB_PERM_SNOOP_ONLY = _EB_SNOOP_FLAG,
    EB_PERM_WRITE_SNOOP = (_EB_WRITE_FLAG | _EB_SNOOP_FLAG)
#endif
};

extern volatile _Alignas(EB_BUFFER_SIZE) uint8_t _eb_memory[EB_BUFFER_SIZE * 2];

/// @brief initialise and start the PIO and DMA interface to the 6502 bus
/// @param pio the pio instance to use
void eb_init(PIO pio);


/// @brief shutdown the 6502 bus interface prior to reset
void eb_shutdown();

/// @brief set the read/write permissions for an address
/// @param address 6502 address
/// @param  perm see enum for possible values
static inline void eb_set_perm_byte(uint16_t address, enum eb_perm perm)
{
    _eb_memory[address * 2 + 1] = perm;
}

/// @brief set the read/write permissions for a range of addresses
/// @param start 6502 starting address
/// @param  perm see enum for possible values
/// @param size number of bytes to set
static inline void eb_set_perm(uint16_t start, enum eb_perm perm, size_t size)
{
    hard_assert(start + size <= EB_BUFFER_SIZE);
    for (size_t i = start; i < start + size; i++)
    {
        eb_set_perm_byte(i, perm);
    }
}

/// @brief get a byte value
/// @param address the 6502 address
/// @return the value of the byte
static inline uint8_t eb_get(uint16_t address)
{
    return _eb_memory[address * 2];
}

/// @brief get a 32 bit value
/// @param address the 6502 address
/// @return the 32 bit value
static inline uint32_t eb_get32(uint16_t address)
{
    uint32_t result =
        (_eb_memory[address * 2] << 24) +
        (_eb_memory[address * 2 + 2] << 16) +
        (_eb_memory[address * 2 + 4] << 8) +
        (_eb_memory[address * 2 + 6] << 0);

    return result;
}

/// @brief set a byte to a new value
/// @param address the 6502 address
/// @param value the new value
static inline void eb_set(uint16_t address, unsigned char value)
{
    _eb_memory[address * 2] = value;
}

/// @brief get a string of chars
/// @param buffer destination buffer
/// @param size number of chars to get
/// @param address 6502 address of source
static inline void eb_get_chars(char *buffer, size_t size, uint16_t address)
{
    for (size_t i = 0; i < size; i++)
    {
        buffer[i] = eb_get(address + i);
    }
}

/// @brief copy a string of chars to memory
/// @param address 6502 destination address
/// @param buffer source
/// @param size number of chars to copy
static inline void eb_set_chars(uint16_t address, const char *buffer, size_t size)
{
    hard_assert(address + size <= EB_BUFFER_SIZE);
    for (size_t i = 0; i < size; i++)
    {
        eb_set(address + i, buffer[i]);
    }
}

/// @brief copy a null terminated string to memory
/// @param address 6502 destination address
/// @param str source
static inline void eb_set_string(uint16_t address, const char *str)
{
    eb_set_chars(address, str, strlen(str));
}


/// @brief copies a value to each location starting at address
/// @param address the address to start at
/// @param c the value to copy
/// @param size the number of loactions to set
static inline void eb_memset(uint16_t address, char c, size_t size)
{
    hard_assert(address + size <= EB_BUFFER_SIZE);
    for (size_t i = address; i < address + size; i++)
    {
        eb_set(i, c);
    }
}

/// @brief get the DMA channel that writes to the event queue
/// @return the DMA channel number
uint eb_get_event_chan();

/*! \brief Set an exclusive iterrupt handler for a 6502 write event 
 *
 * 
 * \param handler The handler to set.
*/
void  eb_set_exclusive_handler(irq_handler_t handler);

/// @brief get the next 6502 address from the event queue
/// @return 16-bit 6502 address, -1 indicates the queue is empty
int eb_get_event();
