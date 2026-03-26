GetLockedJoint
-> no grpc in framework proto

Local C++ build from workspace root
-> Keep main build flow unchanged
-> Source dir stays cpp/
-> Binary dir stays build/
-> VS Code root settings point CMake Tools to cpp/ and build/
-> Local script added: build_cpp_local.ps1
-> Example commands from repo root:
	powershell -ExecutionPolicy Bypass -File .\build_cpp_local.ps1 -Configuration Debug
	powershell -ExecutionPolicy Bypass -File .\build_cpp_local.ps1 -Configuration Release
	powershell -ExecutionPolicy Bypass -File .\build_cpp_local.ps1 -Configuration Debug -CleanCache
-> Current local cmake path:
	C:/Program Files (x86)/Microsoft Visual Studio/18/BuildTools/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe