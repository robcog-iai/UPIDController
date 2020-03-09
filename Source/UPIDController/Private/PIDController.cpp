// Copyright 2017-2020, Institute for Artificial Intelligence - University of Bremen
// Author: Andrei Haidu (http://haidu.eu)

#include "PIDController.h"

// Init ctor
FPIDController::FPIDController(float InP, float InI, float InD, float InMaxOutAbs)
	: P(InP), I(InI), D(InD), MaxOutAbs(InMaxOutAbs)
{
	// Reset errors, bind update function ptr
	FPIDController::Init();
}

// Init 
void FPIDController::Init(float InP, float InI, float InD, float InMaxOutAbs, bool bClearErrors /*= true*/)
{
	P = InP;
	I = InI;
	D = InD;
	MaxOutAbs = InMaxOutAbs;
	// Reset errors, bind update function ptr
	FPIDController::Init(bClearErrors);
}

// Default init
void FPIDController::Init(bool bClearErrors /*= true*/)
{
	if (bClearErrors)
	{
		PrevErr = 0.f;
		IErr = 0.f;
	}

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

// Call the update function pointer
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
