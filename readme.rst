.. _card_alveo_u200:

AMD Alveo U200
--------------

- Card information:
    - Vendor: AMD/Xilinx
    - Name: Alveo U200
    - Ethernet ports: 2x QSFP28
    - PCIe conectors: Edge connector
    - `FPGA Card Website <https://www.xilinx.com/products/boards-and-kits/alveo/u200.html>`_
- FPGA specification:
    - FPGA part number: ``xcu200-fsgd2104-2-e`` (basically same as VU9P)
    - Ethernet Hard IP: CMAC (100G Ethernet)
    - PCIe Hard IP: USP (up to PCIe Gen3 x16)

NDK firmware support
^^^^^^^^^^^^^^^^^^^^

- Ethernet cores that are supported in the NDK firmware:
    - :ref:`CMAC in the Network Module <ndk_intel_net_mod>`
- PCIe cores that are supported in the NDK firmware:
    - :ref:`USP in the PCIe Module <ndk_intel_pcie_mod>`
    - See the ``<NDK-APP_root_directory>/ndk/card/alveo-u200/config/card_conf.tcl`` file for supported PCIe configurations.
- Makefile targets for building the NDK firmware (valid for NDK-APP-Minimal, may vary for other apps):
    - Use ``make 100g2`` command for firmware with 2x100GbE (default).
    - Use ``make 100g0`` command for firmware with CMAC disabled but DMAs and Application core remaining (experimental feature).
- Support for booting the NDK firmware using the nfb-boot tool:
    - NO, use JTAG.

.. note::

    To build the NDK firmware for this card, you must have the Xilinx Vivado installed, including a valid license.
