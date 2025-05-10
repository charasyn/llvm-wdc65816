add_llvm_component_library(LLVMWDC65816Info
  WDC65816TargetInfo.cpp

  LINK_COMPONENTS
  MC
  Support

  ADD_TO_COMPONENT
  WDC65816
)
