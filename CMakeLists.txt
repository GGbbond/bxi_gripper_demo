cmake_minimum_required(VERSION 3.16)
project(bxi_gripper_backend C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

set(BXI_PCI_DRV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/backend/bxi_pci_drv" CACHE PATH
    "Directory containing bxi_pci_drv.h and libbxi_pci_drv.a")
if(NOT EXISTS "${BXI_PCI_DRV_DIR}/bxi_pci_drv.h")
    message(FATAL_ERROR "BXI PCI driver header not found: ${BXI_PCI_DRV_DIR}/bxi_pci_drv.h")
endif()
if(NOT EXISTS "${BXI_PCI_DRV_DIR}/libbxi_pci_drv.a")
    message(FATAL_ERROR "BXI PCI driver library not found: ${BXI_PCI_DRV_DIR}/libbxi_pci_drv.a")
endif()

add_executable(gripper_backend backend/gripper_backend.c)
target_include_directories(gripper_backend PRIVATE "${BXI_PCI_DRV_DIR}")
target_compile_options(gripper_backend PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(gripper_backend PRIVATE
    pthread m "${BXI_PCI_DRV_DIR}/libbxi_pci_drv.a")
