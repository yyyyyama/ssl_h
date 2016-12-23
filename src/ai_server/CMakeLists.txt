file(GLOB_RECURSE SOURCES ./*.cc)
add_library(ai-server-lib ${SOURCES})
add_dependencies(ai-server-lib ssl-protos)
