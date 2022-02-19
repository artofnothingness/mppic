from conans import ConanFile


class MPPIC(ConanFile):
    name = "MPPIC_critics"
    version = "0.3.0"
    requires = (
        "xtensor/0.24.0",
        "catch2/2.13.8",
    )
    generators = "cmake", "gcc", "txt", "cmake_find_package"

