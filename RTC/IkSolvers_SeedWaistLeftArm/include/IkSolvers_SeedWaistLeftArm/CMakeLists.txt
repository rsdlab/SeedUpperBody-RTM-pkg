set(hdrs IkSolvers_SeedWaistLeftArm.h
    PARENT_SCOPE
    WaistLeftArmIkInterfaceSVC_impl.h
    )

install(FILES ${hdrs} DESTINATION ${INC_INSTALL_DIR}/${PROJECT_NAME_LOWER}
    COMPONENT library)

