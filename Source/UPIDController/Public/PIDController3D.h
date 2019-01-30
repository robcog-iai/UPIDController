// Copyright 2019, Institute for Artificial Intelligence - University of Bremen
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
	GENERATED_USTRUCT_BODY()

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

	// Reset error values, bind update function ptr
	void Init();

	// Set PID values, reset error values, bind update function ptr
	void Init(float InP, float InI, float InD, float InMaxOutAbs);

	// Update the PID loop
	FVector Update(const FVector InError, const float InDeltaTime);

	// Update as a PID controller
	FVector UpdateAsPID(const FVector InError, const float InDeltaTime);

	// Update as a P controller
	FVector UpdateAsP(const FVector InError, const float InDeltaTime=0.f);

	// Update as a PD controller
	FVector UpdateAsPD(const FVector InError, const float InDeltaTime);

	// Update as a PI controller
	FVector UpdateAsPI(const FVector InError, const float InDeltaTime);	

private:
	// Previous step error value
	FVector PrevErr;

	// Integral error
	FVector IErr;
};

/* FPIDController inline functions
*****************************************************************************/
FORCEINLINE FPIDController3D::FPIDController3D(float InP, float InI, float InD, float InMaxOutAbs)
	: P(InP), I(InI), D(InD), MaxOutAbs(InMaxOutAbs)
{
	// Reset errors, bind update function ptr
	FPIDController3D::Init();
}

FORCEINLINE void FPIDController3D::Init(float InP, float InI, float InD, float InMaxOutAbs)
{
	P = InP;
	I = InI;
	D = InD;
	MaxOutAbs = InMaxOutAbs;
	// Reset errors, bind update function ptr
	FPIDController3D::Init();
}

FORCEINLINE void FPIDController3D::Init()
{
	PrevErr = FVector(0.f);
	IErr = FVector(0.f);

	// Bind the update type function ptr
	if (P > 0.f && I > 0.f && D > 0.f)
	{
		UpdateFunctionPtr = &FPIDController3D::UpdateAsPID;
	}
	else if (P > 0.f && I > 0.f)
	{
		UpdateFunctionPtr = &FPIDController3D::UpdateAsPI;
	}
	else if (P > 0.f && D > 0.f)
	{
		UpdateFunctionPtr = &FPIDController3D::UpdateAsPD;
	}
	else if (P > 0.f)
	{
		UpdateFunctionPtr = &FPIDController3D::UpdateAsP;
	}
	else
	{
		// Default
		UpdateFunctionPtr = &FPIDController3D::UpdateAsPID;
	}
}

FORCEINLINE FVector FPIDController3D::Update(const FVector InError, const float InDeltaTime)
{
	return (this->*UpdateFunctionPtr)(InError, InDeltaTime);
}

FORCEINLINE FVector FPIDController3D::UpdateAsPID(const FVector InError, const float InDeltaTime)
{
	//if (InDeltaTime == 0.0f || InError.ContainsNaN())
	//{
	//	return FVector(0.0f);
	//}

	// Calculate proportional output
	const FVector POut = P * InError;

	// Calculate integral error / output
	IErr += InDeltaTime * InError;
	const FVector IOut = I * IErr;

	// Calculate the derivative error / output
	const FVector DErr = (InError - PrevErr) / InDeltaTime;
	const FVector DOut = D * DErr;

	// Set previous error
	PrevErr = InError;

	// Calculate the output
	const FVector Out = POut + IOut + DOut;

	// Clamp the vector values
	return Out.BoundToCube(MaxOutAbs);
}

FORCEINLINE FVector FPIDController3D::UpdateAsP(const FVector InError, const float /*InDeltaTime*/)
{
	//if (InError.ContainsNaN())
	//{
	//	return FVector(0.0f);
	//}

	// Calculate proportional output
	const FVector Out = P * InError;

	// Clamp output
	return Out.BoundToCube(MaxOutAbs);
}

FORCEINLINE FVector FPIDController3D::UpdateAsPD(const FVector InError, const float InDeltaTime)
{
	//if (InDeltaTime == 0.0f || InError.ContainsNaN())
	//{
	//	return FVector(0.0f);
	//}

	// Calculate proportional output
	const FVector POut = P * InError;

	// Calculate the derivative error / output
	const FVector DErr = (InError - PrevErr) / InDeltaTime;
	const FVector DOut = D * DErr;

	// Set previous error
	PrevErr = InError;

	// Calculate the output
	const FVector Out = POut + DOut;

	// Clamp output
	return Out.BoundToCube(MaxOutAbs);
}

FORCEINLINE FVector FPIDController3D::UpdateAsPI(const FVector InError, const float InDeltaTime)
{
	//if (InDeltaTime == 0.0f || InError.ContainsNaN())
	//{
	//	return FVector(0.0f);
	//}

	// Calculate proportional output
	const FVector POut = P * InError;

	// Calculate integral error / output
	IErr += InDeltaTime * InError;
	const FVector IOut = I * IErr;

	// Calculate the output
	const FVector Out = POut + IOut;

	// Clamp output
	return Out.BoundToCube(MaxOutAbs);
}
