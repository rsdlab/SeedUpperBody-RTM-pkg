set(hdrs IkSolvers_SeedWaistRightArm.h
    PARENT_SCOPE
    WaistRightArmIkInterfaceSVC_impl.h
    )

install(FILES ${hdrs} DESTINATION ${INC_INSTALL_DIR}/${PROJECT_NAME_LOWER}
    COMPONENT library)

