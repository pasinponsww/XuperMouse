#include "st_i2c.h"

/**
* The implementation in polling based I2C read and write functions
* We have data, len, reg_addr, dev_addr to do I2C read and write
* @date 1/23/2026
*/

namespace MM
{
namespace Stmf4
{
HwI2c::HwI2c(const StI2cParams& params) : _base_addr{params.base_addr}
{
}

bool HwI2c::init()
{
    if (_base_addr == nullptr)
    {
        return false;
    }

    // Reset peripheral
    _base_addr->CR1 &= ~I2C_CR1_PE;

    // Enable peripheral
    _base_addr->CR1 |= I2C_CR1_PE;

    return true;
}

bool HwI2c::mem_read(uint8_t* data, size_t len, const uint8_t reg_addr,
                     uint8_t dev_addr)
{
    if (_base_addr == nullptr)
    {
        return false;
    }

    // Check if init was called
    if (!(_base_addr->CR1 & I2C_CR1_PE))
    {
        return false;
    }

    // F4: Wait for bus idle
    if (_base_addr->SR2 & I2C_SR2_BUSY)
        return false;

    // START, send address (write)
    _base_addr->CR1 |= I2C_CR1_START;
    while (!(_base_addr->SR1 & I2C_SR1_SB)) MM::Utils::DelayUs(1);
    (void)_base_addr->SR1;
    _base_addr->DR = (dev_addr << 1) | 0;
    while (!(_base_addr->SR1 & I2C_SR1_ADDR)) MM::Utils::DelayUs(1);
    (void)_base_addr->SR2;
    while (!(_base_addr->SR1 & I2C_SR1_TXE)) MM::Utils::DelayUs(1);
    _base_addr->DR = reg_addr;
    while (!(_base_addr->SR1 & I2C_SR1_TXE)) MM::Utils::DelayUs(1);

    // Repeated START, send address (read)
    _base_addr->CR1 |= I2C_CR1_START;
    while (!(_base_addr->SR1 & I2C_SR1_SB)) MM::Utils::DelayUs(1);
    (void)_base_addr->SR1;
    _base_addr->DR = (dev_addr << 1) | 1;
    while (!(_base_addr->SR1 & I2C_SR1_ADDR)) MM::Utils::DelayUs(1);
    (void)_base_addr->SR2;
    for (size_t i = 0; i < len; ++i)
    {
        while (!(_base_addr->SR1 & I2C_SR1_RXNE)) MM::Utils::DelayUs(1);
        data[i] = _base_addr->DR;
    }
    _base_addr->CR1 |= I2C_CR1_STOP;
    return true;
}

bool HwI2c::mem_write(const uint8_t* data, size_t len, const uint8_t reg_addr,
                      uint8_t dev_addr)
{
    if (_base_addr == nullptr)
        return false;

    if (!(_base_addr->CR1 & I2C_CR1_PE))
        return false;

    // Wait for bus idle
    if (_base_addr->SR2 & I2C_SR2_BUSY)
        return false;

    // START, send address (write)
    _base_addr->CR1 |= I2C_CR1_START;
    while (!(_base_addr->SR1 & I2C_SR1_SB)) MM::Utils::DelayUs(1);
    (void)_base_addr->SR1;
    _base_addr->DR = (dev_addr << 1) | 0;
    while (!(_base_addr->SR1 & I2C_SR1_ADDR)) MM::Utils::DelayUs(1);
    (void)_base_addr->SR2;
    while (!(_base_addr->SR1 & I2C_SR1_TXE)) MM::Utils::DelayUs(1);
    _base_addr->DR = reg_addr;
    while (!(_base_addr->SR1 & I2C_SR1_TXE)) MM::Utils::DelayUs(1);

    // Write data
    for (size_t i = 0; i < len; ++i)
    {
        _base_addr->DR = data[i];
        while (!(_base_addr->SR1 & I2C_SR1_TXE)) MM::Utils::DelayUs(1);
    }

    // Wait for transfer finished
    while (!(_base_addr->SR1 & I2C_SR1_BTF)) MM::Utils::DelayUs(1);
    _base_addr->CR1 |= I2C_CR1_STOP;
    return true;
}

bool HwI2c::write(const uint8_t* data, size_t len, uint8_t dev_addr)
{
    if (_base_addr == nullptr)
        return false;

    if (!(_base_addr->CR1 & I2C_CR1_PE))
        return false;

    // Wait for bus idle
    if (_base_addr->SR2 & I2C_SR2_BUSY)
        return false;

    // START, send address (write)
    _base_addr->CR1 |= I2C_CR1_START;
    while (!(_base_addr->SR1 & I2C_SR1_SB)) MM::Utils::DelayUs(1);
    (void)_base_addr->SR1;
    _base_addr->DR = (dev_addr << 1) | 0;
    while (!(_base_addr->SR1 & I2C_SR1_ADDR)) MM::Utils::DelayUs(1);
    (void)_base_addr->SR2;

    // Write data
    for (size_t i = 0; i < len; ++i)
    {
        _base_addr->DR = data[i];
        while (!(_base_addr->SR1 & I2C_SR1_TXE)) MM::Utils::DelayUs(1);
    }

    // Wait for transfer finished
    while (!(_base_addr->SR1 & I2C_SR1_BTF)) MM::Utils::DelayUs(1);
    _base_addr->CR1 |= I2C_CR1_STOP;
    return true;
}

bool HwI2c::read(uint8_t* data, size_t len, uint8_t dev_addr)
{
    if (_base_addr == nullptr)
        return false;

    if (!(_base_addr->CR1 & I2C_CR1_PE))
        return false;

    // Wait for bus idle
    if (_base_addr->SR2 & I2C_SR2_BUSY)
        return false;

    // START, send address (read)
    _base_addr->CR1 |= I2C_CR1_START;
    while (!(_base_addr->SR1 & I2C_SR1_SB)) MM::Utils::DelayUs(1);
    (void)_base_addr->SR1;
    _base_addr->DR = (dev_addr << 1) | 1;
    while (!(_base_addr->SR1 & I2C_SR1_ADDR)) MM::Utils::DelayUs(1);
    (void)_base_addr->SR2;

    for (size_t i = 0; i < len; ++i)
    {
        while (!(_base_addr->SR1 & I2C_SR1_RXNE)) MM::Utils::DelayUs(1);
        data[i] = _base_addr->DR;
    }
    _base_addr->CR1 |= I2C_CR1_STOP;
    return true;
}

}  // namespace Stmf4
}  // namespace MM