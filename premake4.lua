lib_sources = {
   "tiny_obj_loader.cc",
   "eson.cc",
}

sources = {
   "obj2eson.cc"
   }

-- premake4.lua
solution "Obj2ESONSolution"
   configurations { "Release", "Debug" }

   if (os.is("windows")) then
      platforms { "x32", "x64" }
   else
      platforms { "native", "x32", "x64" }
   end

   -- A project defines one build target
   project "obj2eson"
      kind "ConsoleApp"
      language "C++"
      files { lib_sources, sources }

      configuration "Debug"
         defines { "DEBUG" } -- -DDEBUG
         flags { "Symbols" }
         targetname "obj2eson_debug"

      configuration "Release"
         -- defines { "NDEBUG" } -- -NDEBUG
         flags { "Symbols", "Optimize" }
         targetname "obj2eson"
