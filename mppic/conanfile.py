from conans import ConanFile


class MPPIC(ConanFile):
    name = "MPPIC"
    version = "0.1"
    requires = (
        "xtensor/0.23.9",
        "catch2/2.13.4",
    )
    generators = "cmake", "gcc", "txt", "cmake_find_package"

