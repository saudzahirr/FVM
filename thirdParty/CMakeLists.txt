# Add fmt library
add_library(fmt STATIC
    fmt-10.2.1/src/format.cc
)

# Specify include directories for fmt
target_include_directories(fmt PUBLIC fmt-10.2.1/include)
