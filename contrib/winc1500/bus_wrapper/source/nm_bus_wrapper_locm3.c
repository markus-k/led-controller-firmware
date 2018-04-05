#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"

#define NM_BUS_MAX_TRX_SZ	4096

tstrNmBusCapabilities egstrNmBusCapabilities =
{
	NM_BUS_MAX_TRX_SZ
};

static sint8 spi_rw(uint8* out, uint8* in, uint16 len) {
  //spi_enable(SPI1);

#if 0
  if (out) {
    uint16_t len2 = len;

    gpio_clear(GPIOA, GPIO4);
    //spi_enable(SPI1);


    while (len2--) {
      spi_send8(SPI1, *out++);
    }

    /* Wait to receive last data */
    while (!(SPI_SR(SPI1) & SPI_SR_RXNE));

    //uint16_t data = SPI_DR(SPI1);

    /* Wait to transmit last data */
    while (!(SPI_SR(SPI1) & SPI_SR_TXE));

    /* Wait until not busy */
    while (SPI_SR(SPI1) & SPI_SR_BSY);

    for (int i = 0; i < 10; i++) {
      __asm__ volatile("nop");
    }

    //spi_clean_disable(SPI1);
    gpio_set(GPIOA, GPIO4);


  }

  if (in) {
    gpio_clear(GPIOA, GPIO4);
    //spi_enable(SPI1);

    while (len--) {
      spi_send8(SPI1, 0);
      *in++ = spi_read8(SPI1);
    }

    /* Wait to receive last data */
    while (!(SPI_SR(SPI1) & SPI_SR_RXNE));

    //uint16_t data = SPI_DR(SPI1);

    /* Wait to transmit last data */
    while (!(SPI_SR(SPI1) & SPI_SR_TXE));

    /* Wait until not busy */
    while (SPI_SR(SPI1) & SPI_SR_BSY);

    for (int i = 0; i < 10; i++) {
      __asm__ volatile("nop");
    }

    gpio_set(GPIOA, GPIO4);
  }
#else
  uint8_t dummy = 0;
  uint8_t skip_in = 0;
  uint8_t skip_out = 0;

  if ((!in && !out) || len == 0) {
    return M2M_ERR_INVALID_ARG;
  }

  if (!out) {
    out = &dummy;
    skip_out = 1;
  }
  if (!in) {
    in = &dummy;
    skip_in = 1;
  }

  gpio_clear(GPIOA, GPIO4);

  while (len) {
    spi_send(SPI1, *out);
    *in = spi_read(SPI1);

    len--;

    if (!skip_in) {
      in++;
    }
    if (!skip_out) {
      out++;
    }
  }

  /* Wait to receive last data */
  //while (!(SPI_SR(SPI1) & SPI_SR_RXNE));

  // empty the queue
  uint16_t data = SPI_DR(SPI1);

  /* Wait to transmit last data */
  while (!(SPI_SR(SPI1) & SPI_SR_TXE));

  /* Wait until not busy */
  while (SPI_SR(SPI1) & SPI_SR_BSY);

  for (int i = 0; i < 10; i++) {
    __asm__ volatile("nop");
  }

  gpio_set(GPIOA, GPIO4);
#endif

  //spi_clean_disable(SPI1);

  return M2M_SUCCESS;
}

sint8 nm_bus_init(void *pvinit) {
  rcc_periph_clock_enable(RCC_SPI1);

#ifdef STM32F1
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5 | GPIO7);
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO4);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO6);

  // set cs high
  gpio_set(GPIOA, GPIO4);

  spi_reset(SPI1);
  spi_init_master(SPI1,
		  SPI_CR1_BAUDRATE_FPCLK_DIV_2,
		  SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
		  SPI_CR1_CPHA_CLK_TRANSITION_1,
		  SPI_CR1_DFF_8BIT,
		  SPI_CR1_MSBFIRST);

  spi_enable_software_slave_management(SPI1);
  spi_set_nss_high(SPI1);

  spi_enable(SPI1);
#elif STM32F3
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO6 | GPIO7);
  gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO5 | GPIO7);

  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4); // nss
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO4);
  // set nss high
  gpio_set(GPIOA, GPIO4);

  spi_reset(SPI1);

  spi_set_master_mode(SPI1);
  spi_set_baudrate_prescaler(SPI1, SPI_CR1_BR_FPCLK_DIV_64);
  spi_set_clock_polarity_0(SPI1);
  spi_set_clock_phase_0(SPI1);
  spi_set_full_duplex_mode(SPI1);
  spi_set_data_size(SPI1, SPI_CR2_DS_8BIT);
  spi_send_msb_first(SPI1);
  spi_set_nss_high(SPI1);
  //  spi_enable_ss_output(SPI1);
  spi_enable_software_slave_management(SPI1);
  spi_fifo_reception_threshold_8bit(SPI1);
  SPI_I2SCFGR(SPI1) &= ~SPI_I2SCFGR_I2SMOD;

  spi_enable(SPI1);
#else
#  error "Unknown platform for setting up spi"
#endif

  // why?
  nm_bsp_reset();
  nm_bsp_sleep(1);

  return M2M_SUCCESS;
}

sint8 nm_bus_ioctl(uint8 cmd, void* pvParameter) {
  uint8 ret = M2M_SUCCESS;

  switch (cmd) {
    case NM_BUS_IOCTL_RW: {
        tstrNmSpiRw *pstrParam = (tstrNmSpiRw *)pvParameter;
        ret = spi_rw(pstrParam->pu8InBuf, pstrParam->pu8OutBuf, pstrParam->u16Sz);
      }
      break;
  default:
    ret = -1;
    M2M_ERR("invalid ioclt cmd\n");
  }

  return ret;
}

sint8 nm_bus_deinit(void) {
  return M2M_SUCCESS;
}
