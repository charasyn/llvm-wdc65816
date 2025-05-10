add_llvm_component_library(LLVMWDC65816Desc
  WDC65816AsmBackend.cpp
  WDC65816ELFObjectWriter.cpp
  WDC65816InstPrinter.cpp
  WDC65816MCAsmInfo.cpp
  WDC65816MCCodeEmitter.cpp
  WDC65816MCTargetDesc.cpp

  LINK_COMPONENTS
  MC
  MCDisassembler
  Support
  TargetParser
  WDC65816Info

  ADD_TO_COMPONENT
  WDC65816
)
