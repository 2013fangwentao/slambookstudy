file(GLOB ccFiles "*.cc")
add_library(
    myslam SHARED
    ${ccFiles}
)
target_link_libraries(myslam ${THIRD_PARTY_LIBS})