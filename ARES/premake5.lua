workspace "ARES"
	architecture "x64"

	configurations
	{
		"Debug", 
		"Release"
	}


outputdir = "%{cfg.buildcfg}-%{cfg.architecture}/"

project "ARES"
	local m_Location = "ARES"
	location ( m_Location )

	language "C++"
	cppdialect "C++latest"

	targetdir ("$(SolutionDir)bin/" .. outputdir)
	objdir ("$(SolutionDir)bin-int/" .. outputdir)

	files
	{
		m_Location .. "/src/**.h",
		m_Location .. "/src/**.cpp"
	}

	includedirs
	{
		"$(SolutionDir)ARES/src/"
	}

	----- ----- ----- ----- ----- ----- CONFIG's ----- ----- ----- ----- ----- ----- 
	filter "configurations:Debug"
		kind "ConsoleApp"
		defines "DEBUG"
		symbols "On"

	filter "configurations:Release"
		kind "ConsoleApp"
		defines "RELEASE"
		optimize "On"