/*
void ads1115_config_set_OS(uint8_t OS)
{
    OS<<15;
}

void ads1115_init(void)
{
    //1)Read Config read


    uint8_t reg[2] =
    {
        (0<<OS_BIT)| (0000<<MUX_BIT) | (<<PGA_BIT)| (<<MODE_BIT),
        (<<DR_BIT)| (<<COMP_MODE_BIT)| (<<COMP_POL_BIT)| (<<COMP_LAT_BIT)| (<<COMP_QUE_BIT),
    }
    I2Ccfx_WriteArray(ADS115_ADR_GND, ADS1115_CONFIG_REG, reg, 2);
}

I2Ccfx_ReadRegistersAtAddress(DS3231_SLAVE_ADDRESS,START_ADDRESS, pDATA, NUMBYTES_TOREAD);
*/
