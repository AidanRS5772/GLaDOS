from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout

class MyProject(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps"

    def requirements(self):
        self.requires("opencv/4.10.0")
        self.requires("eigen/3.4.0")
        self.requires("boost/1.85.0")
        self.requires("rapidcsv/8.83")

    def configure(self):
        self.options["opencv"].contrib = True
        self.options["opencv"].with_qt = False
        self.options["opencv"].with_gtk = True
        self.options["opencv"].with_wayland = False
        

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def layout(self):
        cmake_layout(self)