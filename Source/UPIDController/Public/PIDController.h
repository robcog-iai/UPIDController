// Copyright 2017-2020, Institute for Artificial Intelligence - University of Bremen
// Author: Andrei Haidu (http://haidu.eu)

#pragma once

#include "EngineMinimal.h"
#include "PIDController.generated.h"

/**
* PID Controller
* Error: where you are vs where you want to be
* Derivative: how fast you are approaching, dampening
* Integral: alignment error
*/
USTRUCT(/*BlueprintType*/)
struct UPIDCONTROLLER_API FPIDController 
{
	GENERATED_USTRUCT_BODY()

public:
	// Proportional gain
	UPROPERTY(EditAnywhere, meta = (ClampMin = "0.0", UIMin = "0.0"))
	float P = 0.f;

	// Integral gain
	UPROPERTY(EditAnywhere, meta = (ClampMin = "0.0", UIMin = "0.0"))
	float I = 0.f;;

	// Derivative gain
	UPROPERTY(EditAnywhere, meta = (ClampMin = "0.0", UIMin = "0.0"))
	float D = 0.f;;

	// Max output (as absolute value)
	UPROPERTY(EditAnywhere, meta = (ClampMin = "0.0", UIMin = "0.0"))
	float MaxOutAbs = 0.f;;

	// Default constructor (no initialization)
	FPIDController() { }

	// Constructor with initial value for each component
	FPIDController(float InP, float InI, float InD, float InMaxOutAbs);

	// Update type function pointer variable
	typedef float (FPIDController::*UpdateTypeFunctionPtr)(const float, const float);

	// Update type function ptr
	UpdateTypeFunctionPtr UpdateFunctionPtr;

	//  Set PID values, reset error values, bind update function ptr
	void Init(float InP, float InI, float InD, float InMaxOutAbs, bool bClearErrors = true);

	// Reset error values, bind update function ptr
	void Init(bool bClearErrors = true);

	// Update the PID loop
	float Update(const float InError, const float InDeltaTime);

	// Update as a PID controller
	float UpdateAsPID(const float InError, const float InDeltaTime);

	// Update as a P controller
	float UpdateAsP(const float InError, const float InDeltaTime=0);

	// Update as a PD controller
	float UpdateAsPD(const float InError, const float InDeltaTime);

	// Update as a PI controller
	float UpdateAsPI(const float InError, const float InDeltaTime);
	
private:
	// Previous step error value
	float PrevErr;

	// Integral error
	float IErr;
};

