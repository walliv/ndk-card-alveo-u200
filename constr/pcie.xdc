# pcie.xdc
# Copyright (C) 2023 CESNET z. s. p. o.
# Author(s): Jakub Cabal <cabal@cesnet.cz>
#
# SPDX-License-Identifier: BSD-3-Clause

set_property PACKAGE_PIN AF2     [get_ports {PCIE_RX_P[0]}]
set_property PACKAGE_PIN AF1     [get_ports {PCIE_RX_N[0]}]
set_property PACKAGE_PIN AF7     [get_ports {PCIE_TX_P[0]}]
set_property PACKAGE_PIN AF6     [get_ports {PCIE_TX_N[0]}]
set_property PACKAGE_PIN AG4     [get_ports {PCIE_RX_P[1]}]
set_property PACKAGE_PIN AG3     [get_ports {PCIE_RX_N[1]}]
set_property PACKAGE_PIN AG9     [get_ports {PCIE_TX_P[1]}]
set_property PACKAGE_PIN AG8     [get_ports {PCIE_TX_N[1]}]
set_property PACKAGE_PIN AH2     [get_ports {PCIE_RX_P[2]}]
set_property PACKAGE_PIN AH1     [get_ports {PCIE_RX_N[2]}]
set_property PACKAGE_PIN AH7     [get_ports {PCIE_TX_P[2]}]
set_property PACKAGE_PIN AH6     [get_ports {PCIE_TX_N[2]}]
set_property PACKAGE_PIN AJ4     [get_ports {PCIE_RX_P[3]}]
set_property PACKAGE_PIN AJ3     [get_ports {PCIE_RX_N[3]}]
set_property PACKAGE_PIN AJ9     [get_ports {PCIE_TX_P[3]}]
set_property PACKAGE_PIN AJ8     [get_ports {PCIE_TX_N[3]}]
set_property PACKAGE_PIN AK2     [get_ports {PCIE_RX_P[4]}]
set_property PACKAGE_PIN AK1     [get_ports {PCIE_RX_N[4]}]
set_property PACKAGE_PIN AK7     [get_ports {PCIE_TX_P[4]}]
set_property PACKAGE_PIN AK6     [get_ports {PCIE_TX_N[4]}]
set_property PACKAGE_PIN AL4     [get_ports {PCIE_RX_P[5]}]
set_property PACKAGE_PIN AL3     [get_ports {PCIE_RX_N[5]}]
set_property PACKAGE_PIN AL9     [get_ports {PCIE_TX_P[5]}]
set_property PACKAGE_PIN AL8     [get_ports {PCIE_TX_N[5]}]
set_property PACKAGE_PIN AM2     [get_ports {PCIE_RX_P[6]}]
set_property PACKAGE_PIN AM1     [get_ports {PCIE_RX_N[6]}]
set_property PACKAGE_PIN AM7     [get_ports {PCIE_TX_P[6]}]
set_property PACKAGE_PIN AM6     [get_ports {PCIE_TX_N[6]}]
set_property PACKAGE_PIN AN4     [get_ports {PCIE_RX_P[7]}]
set_property PACKAGE_PIN AN3     [get_ports {PCIE_RX_N[7]}]
set_property PACKAGE_PIN AN9     [get_ports {PCIE_TX_P[7]}]
set_property PACKAGE_PIN AN8     [get_ports {PCIE_TX_N[7]}]
set_property PACKAGE_PIN AP2     [get_ports {PCIE_RX_P[8]}]
set_property PACKAGE_PIN AP1     [get_ports {PCIE_RX_N[8]}]
set_property PACKAGE_PIN AP7     [get_ports {PCIE_TX_P[8]}]
set_property PACKAGE_PIN AP6     [get_ports {PCIE_TX_N[8]}]
set_property PACKAGE_PIN AR4     [get_ports {PCIE_RX_P[9]}]
set_property PACKAGE_PIN AR3     [get_ports {PCIE_RX_N[9]}]
set_property PACKAGE_PIN AR9     [get_ports {PCIE_TX_P[9]}]
set_property PACKAGE_PIN AR8     [get_ports {PCIE_TX_N[9]}]
set_property PACKAGE_PIN AT2     [get_ports {PCIE_RX_P[10]}]
set_property PACKAGE_PIN AT1     [get_ports {PCIE_RX_N[10]}]
set_property PACKAGE_PIN AT7     [get_ports {PCIE_TX_P[10]}]
set_property PACKAGE_PIN AT6     [get_ports {PCIE_TX_N[10]}]
set_property PACKAGE_PIN AU4     [get_ports {PCIE_RX_P[11]}]
set_property PACKAGE_PIN AU3     [get_ports {PCIE_RX_N[11]}]
set_property PACKAGE_PIN AU9     [get_ports {PCIE_TX_P[11]}]
set_property PACKAGE_PIN AU8     [get_ports {PCIE_TX_N[11]}]
set_property PACKAGE_PIN AV2     [get_ports {PCIE_RX_P[12]}]
set_property PACKAGE_PIN AV1     [get_ports {PCIE_RX_N[12]}]
set_property PACKAGE_PIN AV7     [get_ports {PCIE_TX_P[12]}]
set_property PACKAGE_PIN AV6     [get_ports {PCIE_TX_N[12]}]
set_property PACKAGE_PIN AW4     [get_ports {PCIE_RX_P[13]}]
set_property PACKAGE_PIN AW3     [get_ports {PCIE_RX_N[13]}]
set_property PACKAGE_PIN BB5     [get_ports {PCIE_TX_P[13]}]
set_property PACKAGE_PIN BB4     [get_ports {PCIE_TX_N[13]}]
set_property PACKAGE_PIN BA2     [get_ports {PCIE_RX_P[14]}]
set_property PACKAGE_PIN BA1     [get_ports {PCIE_RX_N[14]}]
set_property PACKAGE_PIN BD5     [get_ports {PCIE_TX_P[14]}]
set_property PACKAGE_PIN BD4     [get_ports {PCIE_TX_N[14]}]
set_property PACKAGE_PIN BC2     [get_ports {PCIE_RX_P[15]}]
set_property PACKAGE_PIN BC1     [get_ports {PCIE_RX_N[15]}]
set_property PACKAGE_PIN BF5     [get_ports {PCIE_TX_P[15]}]
set_property PACKAGE_PIN BF4     [get_ports {PCIE_TX_N[15]}]

set_property PACKAGE_PIN BD21    [get_ports {PCIE_SYSRST_N}]
set_property IOSTANDARD LVCMOS12 [get_ports {PCIE_SYSRST_N}]
set_property PULLUP true         [get_ports {PCIE_SYSRST_N}]

set_property PACKAGE_PIN AM11    [get_ports {PCIE_SYSCLK_P}] 
set_property PACKAGE_PIN AM10    [get_ports {PCIE_SYSCLK_N}] 

create_clock -period 10.000 -name pcie_clk_p -waveform {0.000 5.000} [get_ports PCIE_SYSCLK_P]
