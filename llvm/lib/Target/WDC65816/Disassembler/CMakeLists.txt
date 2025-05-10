add_llvm_component_library(LLVMWDC65816Disassembler
  WDC65816Disassembler.cpp

  LINK_COMPONENTS
  CodeGenTypes
  WDC65816Desc
  WDC65816Info
  MCDisassembler
  Support

  ADD_TO_COMPONENT
  WDC65816
)

