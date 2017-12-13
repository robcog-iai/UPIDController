UPIDController
=====

Plugin with a PID controller USTRUCT (`FPIDController`), and a PID controller for `FVector`'s in 3 dimensional space (`FPIDController3D`.

Usage
=====

-   Add the plugin to your project (e.g `MyProject/Plugins/UPIDController`)  
    

-   Add the module dependency to your module (Project, Plugin); In the
    `MyModule.Build.cs` file:  

		PublicDependencyModuleNames.AddRange(  
		new string[]  
		{  
		...  
		"UPIDController",  
		...  
		}  
		);  
    

-   Include `PIDController.h` or `PIDController3D.h` where you plan to use the controller.

### Engine Version 4.18
