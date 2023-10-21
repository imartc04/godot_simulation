from conan import ConanFile


class GodotSimulation(ConanFile):

    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps"

    def requirements(self):
        # self.requires("grpc/[~1.54]")
        #self.requires("boost/[~1.83]")
        self.requires("nlohmann_json/[~3.11]")

    def build_requirements(self):
        self.tool_requires("cmake/3.27.5")
        self.tool_requires("ninja/[~1.11]")
