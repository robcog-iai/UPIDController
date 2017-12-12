// Copyright 2017, Institute for Artificial Intelligence - University of Bremen
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
USTRUCT(BlueprintType)
struct UPIDCONTROLLER_API FPIDController3D
{
	GENERATED_USTRUCT_BODY()

public:
	// Proportional gain
	UPROPERTY(EditAnywhere)
	float P;

	// Integral gain
	UPROPERTY(EditAnywhere)
	float I;

	// Derivative gain
	UPROPERTY(EditAnywhere)
	float D;

	// Max output (as absolute value)
	UPROPERTY(EditAnywhere)
	float MaxOutAbs;

	// Default constructor (no initialization)
	FPIDController3D() { }

	// Constructor with initial value for each component
	FPIDController3D(float InP, float InI, float InD, float InMaxOutAbs);

	// Update the PID loop
	FVector Update(const FVector InError, const float InDeltaTime);

	// Update as a PID controller
	FVector UpdateAsPID(const FVector InError, const float InDeltaTime);

	// Update as a P controller
	FVector UpdateAsP(const FVector InError);

	// Update as a PD controller
	FVector UpdateAsPD(const FVector InError, const float InDeltaTime);

	// Update as a PI controller
	FVector UpdateAsPI(const FVector InError, const float InDeltaTime);

	// Reset error values of the PID
	void Reset();

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
	FPIDController3D::Reset();
}

FORCEINLINE FVector FPIDController3D::Update(const FVector InError, const float InDeltaTime)
{
	if (InDeltaTime == 0.0f || InError.ContainsNaN())
	{
		return FVector(0.0f);
	}

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

FORCEINLINE FVector FPIDController3D::UpdateAsPID(const FVector InError, const float InDeltaTime)
{
	return FPIDController3D::Update(InError, InDeltaTime);
}

FORCEINLINE FVector FPIDController3D::UpdateAsP(const FVector InError)
{
	if (InError.ContainsNaN())
	{
		return FVector(0.0f);
	}

	// Calculate proportional output
	const FVector Out = P * InError;

	// Clamp output
	return Out.BoundToCube(MaxOutAbs);
}

FORCEINLINE FVector FPIDController3D::UpdateAsPD(const FVector InError, const float InDeltaTime)
{
	if (InDeltaTime == 0.0f || InError.ContainsNaN())
	{
		return FVector(0.0f);
	}

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
	if (InDeltaTime == 0.0f || InError.ContainsNaN())
	{
		return FVector(0.0f);
	}

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

FORCEINLINE void FPIDController3D::Reset()
{
	PrevErr = FVector(0.f);
	IErr = FVector(0.f);
}


/* DEPRECATED, moved as USTRUCT */
/**
PID Controller for FVector
Error: where you are vs where you want to be
Derivative: how fast you are approaching, dampening
Integral: alignment error
*/
class UPIDCONTROLLER_API PIDController3D
{
public:
	// Default constructor;
	PIDController3D();

	// Constructor
	PIDController3D(float ProportionalVal, float IntegralVal, float DerivativeVal,
		float OutMaxVal = 0.f, float OutMinVal = 0.f);

	// Destructor
	~PIDController3D();

	// Set all PID values
	void SetValues(float ProportionalVal = 0.f, float IntegralVal = 0.f, float DerivativeVal = 0.f,
		float OutMaxVal = 0.f, float OutMinVal = 0.f);

	// Update the PID loop
	FVector Update(const FVector Error, const float DeltaTime);

	// Update only P
	FVector UpdateAsP(const FVector Error, const float DeltaTime);

	// Update only PD
	FVector UpdateAsPD(const FVector Error, const float DeltaTime);

	// Update only PÍ
	FVector UpdateAsPI(const FVector Error, const float DeltaTime);

	// Reset error values of the PID
	void Reset();

private:
	// Proportional gain
	float P;

	// Integral gain
	float I;

	// Derivative gain
	float D;

	// Output maximul clamping value
	float OutMax;

	// Output minimum claming value
	float OutMin;

	// Previous step error value
	FVector PrevErr;

	// Integral error
	FVector IErr;
};
