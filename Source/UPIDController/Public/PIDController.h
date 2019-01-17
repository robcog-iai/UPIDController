// Copyright 2019, Institute for Artificial Intelligence - University of Bremen
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
	float P;

	// Integral gain
	UPROPERTY(EditAnywhere, meta = (ClampMin = "0.0", UIMin = "0.0"))
	float I;

	// Derivative gain
	UPROPERTY(EditAnywhere, meta = (ClampMin = "0.0", UIMin = "0.0"))
	float D;

	// Max output (as absolute value)
	UPROPERTY(EditAnywhere, meta = (ClampMin = "0.0", UIMin = "0.0"))
	float MaxOutAbs;

	// Default constructor (no initialization)
	FPIDController() { }

	// Constructor with initial value for each component
	FPIDController(float InP, float InI, float InD, float InMaxOutAbs);

	// Update type function pointer variable
	typedef float (FPIDController::*UpdateTypeFunctionPtr)(const float, const float);

	// Update type function ptr
	UpdateTypeFunctionPtr UpdateFunctionPtr;

	// Reset error values, bind update function ptr
	void Init();

	//  Set PID values, reset error values, bind update function ptr
	void Init(float InP, float InI, float InD, float InMaxOutAbs);

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

/* FPIDController inline functions
*****************************************************************************/
FORCEINLINE FPIDController::FPIDController(float InP, float InI, float InD, float InMaxOutAbs)
	: P(InP), I(InI), D(InD), MaxOutAbs(InMaxOutAbs)
{ 
	// Reset errors, bind update function ptr
	FPIDController::Init();
}

FORCEINLINE void FPIDController::Init(float InP, float InI, float InD, float InMaxOutAbs)
{
	P = InP;
	I = InI;
	D = InD;
	MaxOutAbs = InMaxOutAbs;
	// Reset errors, bind update function ptr
	FPIDController::Init();
}

FORCEINLINE void FPIDController::Init()
{
	PrevErr = 0.f;
	IErr = 0.f;

	// Bind the update type function ptr
	if (P > 0.f && I > 0.f && D > 0.f)
	{
		UpdateFunctionPtr = &FPIDController::UpdateAsPID;
	}
	else if (P > 0.f && I > 0.f)
	{
		UpdateFunctionPtr = &FPIDController::UpdateAsPI;
	}
	else if (P > 0.f && D > 0.f)
	{
		UpdateFunctionPtr = &FPIDController::UpdateAsPD;
	}
	else if (P > 0.f)
	{
		UpdateFunctionPtr = &FPIDController::UpdateAsP;
	}
	else
	{
		// Default
		UpdateFunctionPtr = &FPIDController::UpdateAsPID;
	}	
}

FORCEINLINE float FPIDController::Update(const float InError, const float InDeltaTime)
{
	return (this->*UpdateFunctionPtr)(InError, InDeltaTime);
}

FORCEINLINE float FPIDController::UpdateAsPID(const float InError, const float InDeltaTime)
{
	//if (InDeltaTime == 0.0f || FMath::IsNaN(InError))
	//{
	//	return 0.0f;
	//}

	// Calculate proportional output
	const float POut = P * InError;

	// Calculate integral error / output
	IErr += InDeltaTime * InError;
	const float IOut = I * IErr;

	// Calculate the derivative error / output
	const float DErr = (InError - PrevErr) / InDeltaTime;
	const float DOut = D * DErr;

	// Set previous error
	PrevErr = InError;

	// Calculate the output
	const float Out = POut + IOut + DOut;
	
	// Clamp output
	return FMath::Clamp(Out, -MaxOutAbs, MaxOutAbs);	
}

FORCEINLINE float FPIDController::UpdateAsP(const float InError, const float /*InDeltaTime*/)
{
	//if (FMath::IsNaN(InError))
	//{
	//	return 0.0f;
	//}

	// Calculate proportional output
	const float Out = P * InError;

	// Clamp output
	return FMath::Clamp(Out, -MaxOutAbs, MaxOutAbs);
}

FORCEINLINE float FPIDController::UpdateAsPD(const float InError, const float InDeltaTime)
{
	//if (InDeltaTime == 0.0f || FMath::IsNaN(InError))
	//{
	//	return 0.0f;
	//}

	// Calculate proportional output
	const float POut = P * InError;

	// Calculate the derivative error / output
	const float DErr = (InError - PrevErr) / InDeltaTime;
	const float DOut = D * DErr;

	// Set previous error
	PrevErr = InError;

	// Calculate the output
	const float Out = POut + DOut;

	// Clamp output
	return FMath::Clamp(Out, -MaxOutAbs, MaxOutAbs);
}

FORCEINLINE float FPIDController::UpdateAsPI(const float InError, const float InDeltaTime)
{
	//if (InDeltaTime == 0.0f || FMath::IsNaN(InError))
	//{
	//	return 0.0f;
	//}

	// Calculate proportional output
	const float POut = P * InError;

	// Calculate integral error / output
	IErr += InDeltaTime * InError;
	const float IOut = I * IErr;

	// Calculate the output
	const float Out = POut + IOut;

	// Clamp output
	return FMath::Clamp(Out, -MaxOutAbs, MaxOutAbs);
}
