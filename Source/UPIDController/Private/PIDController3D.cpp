// Copyright 2017-2019, Institute for Artificial Intelligence - University of Bremen
// Author: Andrei Haidu (http://haidu.eu)

#include "PIDController3D.h"

// Init ctor
FPIDController3D::FPIDController3D(float InP, float InI, float InD, float InMaxOutAbs)
	: P(InP), I(InI), D(InD), MaxOutAbs(InMaxOutAbs)
{
	// Reset errors, bind update function ptr
	FPIDController3D::Init();
}

// Init
void FPIDController3D::Init(float InP, float InI, float InD, float InMaxOutAbs, bool bClearErrors /*= true*/)
{
	P = InP;
	I = InI;
	D = InD;
	MaxOutAbs = InMaxOutAbs;
	// Reset errors, bind update function ptr
	FPIDController3D::Init(bClearErrors);
}

// Default init
void FPIDController3D::Init(bool bClearErrors /*= true*/)
{
	if (bClearErrors)
	{
		PrevErr = FVector(0.f);
		IErr = FVector(0.f);
	}

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


// Call the update function pointer
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