// Copyright 2017-2020, Institute for Artificial Intelligence - University of Bremen
// Author: Andrei Haidu (http://haidu.eu)

#pragma once

#include "EngineMinimal.h"
#include "PIDController3D.generated.h"

/**
* PID Controller for FVector
* Error: where you are vs where you want to be
* Derivative: how fast you are approaching, dampening
* Integral: alignment error
*/
USTRUCT(/*BlueprintType*/)
struct UPIDCONTROLLER_API FPIDController3D
{
	GENERATED_BODY()

public:
	// Proportional gain
	UPROPERTY(EditAnywhere, meta = (ClampMin = "0.0", UIMin = "0.0"))
	float P = 0.f;

	// Integral gain
	UPROPERTY(EditAnywhere, meta = (ClampMin = "0.0", UIMin = "0.0"))
	float I = 0.f;

	// Derivative gain
	UPROPERTY(EditAnywhere, meta = (ClampMin = "0.0", UIMin = "0.0"))
	float D = 0.f;

	// Max output (as absolute value)
	UPROPERTY(EditAnywhere, meta = (ClampMin = "0.0", UIMin = "0.0"))
	float MaxOutAbs = 0.f;

	// Default constructor (no initialization)
	FPIDController3D() { }

	// Constructor with initial value for each component
	FPIDController3D(float InP, float InI, float InD, float InMaxOutAbs);

	// Update type function pointer variable
	typedef FVector (FPIDController3D::*UpdateTypeFunctionPtr)(const FVector, const float);

	// Update type function ptr
	UpdateTypeFunctionPtr UpdateFunctionPtr;

	// Set PID values, reset error values, bind update function ptr
	void Init(float InP, float InI, float InD, float InMaxOutAbs, bool bClearErrors = true);

	// Reset error values, bind update function ptr
	void Init(bool bClearErrors = true);

	// Update the PID loop
	/*FORCEINLINE*/ FVector Update(const FVector InError, const float InDeltaTime);

	// Update as a PID controller
	/*FORCEINLINE*/ FVector UpdateAsPID(const FVector InError, const float InDeltaTime);

	// Update as a P controller
	/*FORCEINLINE*/ FVector UpdateAsP(const FVector InError, const float InDeltaTime=0.f);

	// Update as a PD controller
	/*FORCEINLINE*/ FVector UpdateAsPD(const FVector InError, const float InDeltaTime);

	// Update as a PI controller
	/*FORCEINLINE*/ FVector UpdateAsPI(const FVector InError, const float InDeltaTime);

private:
	// Previous step error value
	FVector PrevErr;

	// Integral error
	FVector IErr;
};

