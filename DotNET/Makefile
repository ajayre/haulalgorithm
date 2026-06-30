# HaulBuilder — automated build and test (Windows + macOS).
# Requires: .NET 8 SDK (dotnet CLI). On Windows, use GNU Make from Git Bash,
# Visual Studio, or: winget install GnuWin32.Make

DOTNET       ?= dotnet
SLN          := HaulBuilder.sln
CONFIG       ?= Release
BUILD_PROPS  := -c $(CONFIG) --nologo -v m

.PHONY: all help restore build debug test test-fast test-verbose clean publish verify

help:
	@echo "HaulBuilder build targets:"
	@echo "  make all          restore, build (Release), and test"
	@echo "  make restore      dotnet restore"
	@echo "  make build        Release build"
	@echo "  make debug        Debug build"
	@echo "  make test         run tests (Release)"
	@echo "  make test-fast    run tests excluding Slow category (~2 min)"
	@echo "  make test-verbose run tests with normal verbosity"
	@echo "  make verify       build + test (CI gate)"
	@echo "  make clean        remove build outputs"
	@echo "  make publish      publish HaulBuilder library to artifacts/"

all: verify

restore:
	$(DOTNET) restore $(SLN) --nologo -v m

build: native restore
	$(DOTNET) build $(SLN) -c $(CONFIG) --nologo -v m --no-restore

native:
	powershell -NoProfile -ExecutionPolicy Bypass -File Native/build_native.ps1 -Configuration $(CONFIG)

debug:
	$(DOTNET) build $(SLN) -c Debug --nologo -v m

test: build
	$(DOTNET) test $(SLN) $(BUILD_PROPS) --no-build

test-fast: build
	$(DOTNET) test $(SLN) $(BUILD_PROPS) --no-build --filter "Category!=Slow"

test-verbose: build
	$(DOTNET) test $(SLN) $(BUILD_PROPS) --no-build -v n

verify: build test

clean:
	$(DOTNET) clean $(SLN) -c Debug --nologo
	$(DOTNET) clean $(SLN) -c Release --nologo
	-$(RM) -rf artifacts

publish: build
	$(DOTNET) publish HaulBuilder/HaulBuilder.csproj $(BUILD_PROPS) \
		--no-restore -o artifacts/HaulBuilder
