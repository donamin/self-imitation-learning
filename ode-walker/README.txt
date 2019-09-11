IMPORTANT INFO:
The project works with Microsoft Visual Studio 2017.

The Visual studio project does not maintain all the commands when transferred. To get the project working, go to the project "SCA" and Properties -> Configuration Properties -> Debugging -> Command and insert "$(ProjectDir)/drawstuffrenderer.exe". You must also change the target platform version to your Windows version. To do that, go to each project's Properties -> Configuration Properties -> General -> Target Platform Version, and change the version to your Windows version.

The main file handling everything is "OdeSimpleWalker.cpp".
