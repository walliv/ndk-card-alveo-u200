-- fpga.vhd: Alveo U200 board top level entity and architecture
-- Copyright (C) 2022 CESNET z. s. p. o.
-- Author(s): Jakub Cabal <cabal@cesnet.cz>
--
-- SPDX-License-Identifier: BSD-3-Clause

library ieee;
library unisim;
library xpm;

use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.combo_const.all;
use work.combo_user_const.all;

use work.math_pack.all;
use work.type_pack.all;
use work.dma_bus_pack.all;

use unisim.vcomponents.all;

entity FPGA is
port (
    -- PCIe
    PCIE_SYSCLK_P       : in    std_logic;
    PCIE_SYSCLK_N       : in    std_logic;
    PCIE_SYSRST_N       : in    std_logic;
    PCIE_RX_P           : in    std_logic_vector(PCIE_LANES -1 downto 0);
    PCIE_RX_N           : in    std_logic_vector(PCIE_LANES -1 downto 0);
    PCIE_TX_P           : out   std_logic_vector(PCIE_LANES -1 downto 0);
    PCIE_TX_N           : out   std_logic_vector(PCIE_LANES -1 downto 0);

    -- 156.250 MHz external clock 
    SYSCLK_P            : in    std_logic;
    SYSCLK_N            : in    std_logic;

    -- QSFP control
    --QSFP0_SCL           : inout std_logic;
    --QSFP0_SDA           : inout std_logic;
    QSFP0_LPMODE        : out   std_logic;
    QSFP0_RESET_N       : out   std_logic;
    QSFP0_MODPRS_N      : in    std_logic;
    QSFP0_INT_N         : in    std_logic;

    --QSFP1_SCL           : inout std_logic;
    --QSFP1_SDA           : inout std_logic;
    QSFP1_LPMODE        : out   std_logic;
    QSFP1_RESET_N       : out   std_logic;
    QSFP1_MODPRS_N      : in    std_logic;
    QSFP1_INT_N         : in    std_logic;

    -- QSFP data
    QSFP0_REFCLK_P      : in    std_logic;
    QSFP0_REFCLK_N      : in    std_logic;
    QSFP0_RX_P          : in    std_logic_vector(3 downto 0);
    QSFP0_RX_N          : in    std_logic_vector(3 downto 0);
    QSFP0_TX_P          : out   std_logic_vector(3 downto 0);
    QSFP0_TX_N          : out   std_logic_vector(3 downto 0);

    QSFP1_REFCLK_P      : in    std_logic;
    QSFP1_REFCLK_N      : in    std_logic;
    QSFP1_RX_P          : in    std_logic_vector(3 downto 0);
    QSFP1_RX_N          : in    std_logic_vector(3 downto 0);
    QSFP1_TX_P          : out   std_logic_vector(3 downto 0);
    QSFP1_TX_N          : out   std_logic_vector(3 downto 0);

    LED_STATUS          : out   std_logic_vector(2 downto 0)
);
end entity;

architecture FULL of FPGA is

    -- DMA debug parameters
    constant DMA_GEN_LOOP_EN     : boolean := true;

    constant PCIE_CLKS           : integer := 1;
    constant PCIE_CONS           : integer := 1;
    constant MISC_IN_WIDTH       : integer := 4;
    constant MISC_OUT_WIDTH      : integer := 4;
    constant ETH_LANES           : integer := 4;
    constant DMA_MODULES         : integer := PCIE_ENDPOINTS;
    constant DMA_ENDPOINTS       : integer := PCIE_ENDPOINTS;
    constant ETH_LANE_MAP        : integer_vector(2*ETH_LANES-1 downto 0) := (3, 2, 1, 0, 3, 2, 1, 0);
    constant ETH_LANE_RXPOLARITY : std_logic_vector(2*ETH_LANES-1 downto 0) := "00000000";
    constant ETH_LANE_TXPOLARITY : std_logic_vector(2*ETH_LANES-1 downto 0) := "00000000";
    constant DEVICE              : string  := "ULTRASCALE";
    
    signal sysclk_ibuf      : std_logic;
    signal sysclk_bufg      : std_logic;
    signal sysrst_cnt       : unsigned(4 downto 0) := (others => '0');
    signal sysrst           : std_logic := '1';
    
    signal eth_refclk_p     : std_logic_vector(2-1 downto 0);
    signal eth_refclk_n     : std_logic_vector(2-1 downto 0);
    signal eth_rx_p         : std_logic_vector(2*ETH_LANES-1 downto 0);
    signal eth_rx_n         : std_logic_vector(2*ETH_LANES-1 downto 0);
    signal eth_tx_p         : std_logic_vector(2*ETH_LANES-1 downto 0);
    signal eth_tx_n         : std_logic_vector(2*ETH_LANES-1 downto 0);

    signal qsfp_lpmode      : std_logic_vector(2-1 downto 0) := (others => '1');
    signal qsfp_reset_n     : std_logic_vector(2-1 downto 0) := (others => '0');
    signal qsfp_scl         : std_logic_vector(2-1 downto 0) := (others => 'Z');
    signal qsfp_sda         : std_logic_vector(2-1 downto 0) := (others => 'Z');
    signal qsfp_modprs_n    : std_logic_vector(2-1 downto 0);
    signal qsfp_int_n       : std_logic_vector(2-1 downto 0);
    
    signal misc_in          : std_logic_vector(MISC_IN_WIDTH-1 downto 0) := (others => '0');
    signal misc_out         : std_logic_vector(MISC_OUT_WIDTH-1 downto 0);

begin

    sysclk_ibuf_i : IBUFDS
    port map (
        I  => SYSCLK_P,
        IB => SYSCLK_N,
        O  => sysclk_ibuf
    );

    sysclk_bufg_i : BUFG
    port map (
        I => sysclk_ibuf,
        O => sysclk_bufg
    );

    -- reset after power up
    process(sysclk_bufg)
    begin
        if rising_edge(sysclk_bufg) then
            if (sysrst_cnt(sysrst_cnt'high) = '0') then
                sysrst_cnt <= sysrst_cnt + 1;
            end if;
            sysrst <= not sysrst_cnt(sysrst_cnt'high);
        end if;
    end process;

    -- QSFP MAPPING ------------------------------------------------------------
    eth_refclk_p <= QSFP1_REFCLK_P & QSFP0_REFCLK_P; 
    eth_refclk_n <= QSFP1_REFCLK_N & QSFP0_REFCLK_N;

    eth_rx_p <= QSFP1_RX_P & QSFP0_RX_P;
    eth_rx_n <= QSFP1_RX_N & QSFP0_RX_N;

    net_arch_empty_g: if (NET_MOD_ARCH /= "EMPTY") generate
        QSFP1_TX_P <= eth_tx_p(2*ETH_LANES-1 downto 1*ETH_LANES);
        QSFP1_TX_N <= eth_tx_n(2*ETH_LANES-1 downto 1*ETH_LANES);
        QSFP0_TX_P <= eth_tx_p(1*ETH_LANES-1 downto 0*ETH_LANES);
        QSFP0_TX_N <= eth_tx_n(1*ETH_LANES-1 downto 0*ETH_LANES);

        QSFP1_LPMODE  <= qsfp_lpmode(1);
        QSFP1_RESET_N <= qsfp_reset_n(1);
        QSFP0_LPMODE  <= qsfp_lpmode(0);
        QSFP0_RESET_N <= qsfp_reset_n(0);
    end generate;

    --QSFP1_SCL     <= qsfp_scl(1);
    --QSFP1_SDA     <= qsfp_sda(1);
    --QSFP0_SCL     <= qsfp_scl(0);
    --QSFP0_SDA     <= qsfp_sda(0);

    qsfp_modprs_n <= QSFP1_MODPRS_N & QSFP0_MODPRS_N;
    qsfp_int_n    <= QSFP1_INT_N & QSFP0_INT_N;
 
    -- FPGA COMMON -------------------------------------------------------------
    cm_i : entity work.FPGA_COMMON
    generic map (
        SYSCLK_FREQ             => 250, -- PCIe AXI clock frequency
        USE_PCIE_CLK            => true,
        
        PCIE_LANES              => PCIE_LANES,
        PCIE_CLKS               => PCIE_CLKS,
        PCIE_CONS               => PCIE_CONS,

        ETH_CORE_ARCH           => NET_MOD_ARCH,
        ETH_PORTS               => ETH_PORTS,
        ETH_PORT_SPEED          => ETH_PORT_SPEED,
        ETH_PORT_CHAN           => ETH_PORT_CHAN,
        ETH_LANES               => ETH_LANES,
        ETH_LANE_MAP            => ETH_LANE_MAP(ETH_PORTS*ETH_LANES-1 downto 0),
        ETH_LANE_RXPOLARITY     => ETH_LANE_RXPOLARITY(ETH_PORTS*ETH_LANES-1 downto 0),
        ETH_LANE_TXPOLARITY     => ETH_LANE_TXPOLARITY(ETH_PORTS*ETH_LANES-1 downto 0),

        QSFP_PORTS              => ETH_PORTS,
        QSFP_I2C_PORTS          => ETH_PORTS,
        ETH_PORT_LEDS           => 2, -- fake leds

        STATUS_LEDS             => 3,

        MISC_IN_WIDTH           => MISC_IN_WIDTH,
        MISC_OUT_WIDTH          => MISC_OUT_WIDTH,

        PCIE_ENDPOINTS          => PCIE_ENDPOINTS,
        PCIE_ENDPOINT_TYPE      => PCIE_MOD_ARCH,
        PCIE_ENDPOINT_MODE      => PCIE_ENDPOINT_MODE,

        DMA_ENDPOINTS           => DMA_ENDPOINTS,
        DMA_MODULES             => DMA_MODULES,

        DMA_RX_CHANNELS         => DMA_RX_CHANNELS/DMA_MODULES,
        DMA_TX_CHANNELS         => DMA_TX_CHANNELS/DMA_MODULES,

        BOARD                   => BOARD,
        DEVICE                  => DEVICE,

        --AMM_FREQ_KHZ            => DDR_FREQ,
        --MEM_PORTS               => DDR_PORTS,
        --MEM_ADDR_WIDTH          => AMM_ADDR_WIDTH,
        --MEM_DATA_WIDTH          => AMM_DATA_WIDTH,
        --MEM_BURST_WIDTH         => AMM_BURST_COUNT_WIDTH,

        DMA_GEN_LOOP_EN         => DMA_GEN_LOOP_EN
    )
    port map(
        SYSCLK                  => sysclk_bufg,
        SYSRST                  => sysrst,

        PCIE_SYSCLK_P(0)        => PCIE_SYSCLK_P,
        PCIE_SYSCLK_N(0)        => PCIE_SYSCLK_N,
        PCIE_SYSRST_N(0)        => PCIE_SYSRST_N,
        PCIE_RX_P               => PCIE_RX_P,
        PCIE_RX_N               => PCIE_RX_N,
        PCIE_TX_P               => PCIE_TX_P,
        PCIE_TX_N               => PCIE_TX_N,

        ETH_REFCLK_P            => eth_refclk_p(ETH_PORTS-1 downto 0),
        ETH_REFCLK_N            => eth_refclk_n(ETH_PORTS-1 downto 0),

        ETH_RX_P                => eth_rx_p(ETH_PORTS*ETH_LANES-1 downto 0),
        ETH_RX_N                => eth_rx_n(ETH_PORTS*ETH_LANES-1 downto 0),
        ETH_TX_P                => eth_tx_p(ETH_PORTS*ETH_LANES-1 downto 0),
        ETH_TX_N                => eth_tx_n(ETH_PORTS*ETH_LANES-1 downto 0),

        ETH_LED_R               => open,
        ETH_LED_G               => open,

        QSFP_I2C_SCL            => open,--qsfp_scl(ETH_PORTS-1 downto 0),
        QSFP_I2C_SDA            => open,--qsfp_sda(ETH_PORTS-1 downto 0),

        QSFP_MODSEL_N           => open,
        QSFP_LPMODE             => qsfp_lpmode(ETH_PORTS-1 downto 0),
        QSFP_RESET_N            => qsfp_reset_n(ETH_PORTS-1 downto 0),
        QSFP_MODPRS_N           => qsfp_modprs_n(ETH_PORTS-1 downto 0),
        QSFP_INT_N              => qsfp_int_n(ETH_PORTS-1 downto 0),

        STATUS_LED_G            => LED_STATUS,
        STATUS_LED_R            => open,

        --PCIE_CLK                => pcie_clk,
        --PCIE_RESET              => pcie_reset,
    --
        --BOOT_MI_CLK             => boot_mi_clk,
        --BOOT_MI_RESET           => boot_mi_reset,
        --BOOT_MI_DWR             => boot_mi_dwr,
        --BOOT_MI_ADDR            => boot_mi_addr,
        --BOOT_MI_RD              => boot_mi_rd,
        --BOOT_MI_WR              => boot_mi_wr,
        --BOOT_MI_BE              => boot_mi_be,
        --BOOT_MI_DRD             => boot_mi_drd,
        --BOOT_MI_ARDY            => boot_mi_ardy,
        --BOOT_MI_DRDY            => boot_mi_drdy,

        --MEM_CLK                 => ddr_ui_clk,
        --MEM_RST                 => ddr_ui_clk_sync_rst,
--
        --MEM_AVMM_READY          => mem_avmm_ready,
        --MEM_AVMM_READ           => mem_avmm_read,
        --MEM_AVMM_WRITE          => mem_avmm_write,
        --MEM_AVMM_ADDRESS        => mem_avmm_address,
        --MEM_AVMM_BURSTCOUNT     => mem_avmm_burstcount,
        --MEM_AVMM_WRITEDATA      => mem_avmm_writedata,
        --MEM_AVMM_READDATA       => mem_avmm_readdata,
        --MEM_AVMM_READDATAVALID  => mem_avmm_readdatavalid,
--
        --EMIF_RST_REQ            => ddr_areset,
        --EMIF_RST_DONE           => ddr_init_calib_complete,
        --EMIF_CAL_SUCCESS        => ddr_init_calib_complete,
        --EMIF_ECC_USR_INT        => (others => '0'),
        --EMIF_CAL_FAIL           => (others => '0'),
        --MEM_REFR_ACK            => (others => '1'),

        MISC_IN                 => misc_in,
        MISC_OUT                => misc_out
    );

end architecture;
