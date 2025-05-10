add_llvm_component_library(LLVMWDC65816AsmParser
  WDC65816AsmParser.cpp

  LINK_COMPONENTS
  CodeGenTypes
  WDC65816CodeGen
  WDC65816Info
  MC
  MCParser
  Support

  ADD_TO_COMPONENT
  WDC65816
  )
