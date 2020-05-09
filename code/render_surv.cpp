#include "common_surv.h"
#include "render_surv.h"
#include "math_surv.h"
#include "stdlib.h"
#define MAX_VERTICES (1UL << 16UL)
#define VerticesPerQuad 6

u32 VertexCount = 0;
b32 RenderBatching;
b32 RenderBatchInit;
f32* VertexPosCurrent;
f32* VertexPosBufferData;
f32* VertexColorCurrent;
f32* VertexColorBufferData;
f32* VertexTexCurrent;
f32* VertexTexBufferData;
GLuint BatchVertexArray, VertexColorVBO, VertexPosVBO, VertexTexVBO;

GLuint CreateGLShader(const char* ShaderSource, GLenum ShaderType)
{
	u32 Shader = glCreateShader(ShaderType);
	glShaderSource(Shader, 1, &ShaderSource, NULL);
	glCompileShader(Shader);
	GLint Success;
	char InfoLog[512];
	glGetShaderiv(Shader, GL_COMPILE_STATUS, &Success);
	if(!Success)
	{
		glGetShaderInfoLog(Shader, 512, 0, InfoLog);
		InvalidCodePath;
	}
	return Shader;
}

GLuint CreateVFShader(const char* VertexSource, const char* FragSource)
{
	GLuint VShader = CreateGLShader(VertexSource, GL_VERTEX_SHADER);
	GLuint FShader = CreateGLShader(FragSource, GL_FRAGMENT_SHADER);
	GLuint Program = glCreateProgram();
	glAttachShader(Program, VShader);
	glAttachShader(Program, FShader);
	glLinkProgram(Program);
	glDeleteShader(VShader);
	glDeleteShader(FShader);
	GLint Success;
	char InfoLog[512];
	glGetProgramiv(Program, GL_LINK_STATUS, &Success);
	if(!Success)
	{
		glGetProgramInfoLog(Program, 512, 0, InfoLog);
		  InvalidCodePath;
	}
	return Program;
}

void FlushVertexData()
{
	glBindVertexArray(BatchVertexArray);

#if 1
	glBindBuffer(GL_ARRAY_BUFFER, VertexPosVBO);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(f32) * VertexCount << 1, VertexPosBufferData);
	glBindBuffer(GL_ARRAY_BUFFER, VertexColorVBO);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(f32) * VertexCount << 2, VertexColorBufferData);
	glBindBuffer(GL_ARRAY_BUFFER, VertexTexVBO);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(f32) * VertexCount << 1, VertexTexBufferData);
#endif
	
	glDrawArrays(GL_TRIANGLES, 0, (GLsizei)(VertexCount));
	VertexCount = 0;
	VertexPosCurrent = VertexPosBufferData;
	VertexColorCurrent = VertexColorBufferData;
	VertexTexCurrent = VertexTexBufferData;
}

void StartRenderBatch()
{
	Assert(!RenderBatching);
	if(!RenderBatchInit)
	{
		RenderBatchInit = true;

		glGenVertexArrays(1, &BatchVertexArray);
		glBindVertexArray(BatchVertexArray);
		glGenBuffers(1, &VertexPosVBO);
		glGenBuffers(1, &VertexColorVBO);
		glGenBuffers(1, &VertexTexVBO);

		u32 VertexPosSize = MAX_VERTICES * 2 * sizeof(f32);
		u32 VertexColorSize = MAX_VERTICES * 4 * sizeof(f32);
		u32 VertexTexSize = MAX_VERTICES * 2 * sizeof(f32);

#if 0
		GLbitfield FieldMap = GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT;

		// Buffer creation flags
		GLbitfield FieldCreate = FieldMap | GL_DYNAMIC_STORAGE_BIT;

		glBindBuffer(GL_ARRAY_BUFFER, VertexPosVBO);
		glBufferStorage(GL_ARRAY_BUFFER, VertexPosSize, 0, FieldCreate);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);

		// We map the VBO GPU address to our client address space. 
		VertexPosBufferData = (float*)glMapBufferRange(GL_ARRAY_BUFFER, 0, VertexPosSize, FieldMap);
		Assert(VertexPosBufferData);
		VertexPosCurrent  = VertexPosBufferData;

		glBindBuffer(GL_ARRAY_BUFFER, VertexColorVBO);
		glBufferStorage(GL_ARRAY_BUFFER, VertexColorSize, 0, FieldCreate);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, 0);
		VertexColorBufferData = (float*)glMapBufferRange(GL_ARRAY_BUFFER, 0, VertexColorSize, FieldMap);
		Assert(VertexColorBufferData);
		VertexColorCurrent = VertexColorBufferData;

		glBindBuffer(GL_ARRAY_BUFFER, VertexTexVBO);
		glBufferStorage(GL_ARRAY_BUFFER, VertexTexSize, 0, FieldCreate);
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, 0);
		VertexTexBufferData = (float*)glMapBufferRange(GL_ARRAY_BUFFER, 0, VertexTexSize, FieldMap);
		Assert(VertexTexBufferData);
		VertexTexCurrent = VertexTexBufferData;
#else
		VertexPosBufferData= (f32*)malloc(VertexPosSize);
		VertexColorBufferData= (f32*)malloc(VertexColorSize);
		VertexPosCurrent = VertexPosBufferData;
		VertexColorCurrent = VertexColorBufferData;
		VertexTexCurrent = VertexTexBufferData;

		glBindBuffer(GL_ARRAY_BUFFER, VertexPosVBO);
		glBufferData(GL_ARRAY_BUFFER, VertexPosSize, 0, GL_DYNAMIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(f32), 0);

		glBindBuffer(GL_ARRAY_BUFFER, VertexColorVBO);
		glBufferData(GL_ARRAY_BUFFER, VertexColorSize, 0, GL_DYNAMIC_DRAW);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 4*sizeof(f32), 0);
#endif
	}
	Assert(VertexPosCurrent == VertexPosBufferData);
	Assert(VertexColorCurrent == VertexColorBufferData);
	Assert(VertexTexCurrent == VertexTexBufferData);
	RenderBatching = true;
}

inline void PushVertex(f32 X, f32 Y, f32 ColorR, f32 ColorG, f32 ColorB, f32 ColorA)
{
	VertexPosCurrent[0] = X;
	VertexPosCurrent[1] = Y; 
	VertexPosCurrent += 2;

	VertexColorCurrent[0] = ColorR;
	VertexColorCurrent[1] = ColorG;
	VertexColorCurrent[2] = ColorB;
	VertexColorCurrent[3] = ColorA;
	VertexColorCurrent += 4;

	VertexCount++;
}

void RenderQuad(f32 X, f32 Y, f32 Width, f32 Height, f32 Rotation, f32 ColorR, f32 ColorG, f32 ColorB, f32 ColorA)
{
	Assert(RenderBatching);

	if(VertexCount + VerticesPerQuad > MAX_VERTICES)
	{
		FlushVertexData();
	}
	f32 CosVal = Cos(Rotation);
	f32 SinVal = Sin(Rotation);

	f32 RightX, TopY, LeftX, BottomY;

	RightX  = Width/2  ;
	TopY    = Height/2 ;
	LeftX   = -Width/2 ;
	BottomY = -Height/2;

	PushVertex(X + RightX * CosVal - TopY * SinVal,      Y + TopY * CosVal + RightX * SinVal   , ColorR, ColorG, ColorB, ColorA);
	PushVertex(X + RightX * CosVal - BottomY * SinVal,   Y + BottomY * CosVal + RightX * SinVal, ColorR, ColorG, ColorB, ColorA);
	PushVertex(X + LeftX * CosVal - TopY* SinVal,        Y + TopY* CosVal + LeftX * SinVal     , ColorR, ColorG, ColorB, ColorA);
	PushVertex(X + RightX * CosVal - BottomY * SinVal,   Y + BottomY * CosVal + RightX * SinVal, ColorR, ColorG, ColorB, ColorA);
	PushVertex(X + LeftX * CosVal - BottomY * SinVal,    Y + BottomY * CosVal + LeftX * SinVal , ColorR, ColorG, ColorB, ColorA);
	PushVertex(X + LeftX * CosVal - TopY * SinVal,       Y + TopY * CosVal + LeftX * SinVal    , ColorR, ColorG, ColorB, ColorA);
}

void RenderCircle(f32 X, f32 Y, f32 Radius, f32 ColorR, f32 ColorG, f32 ColorB, f32 ColorA, i32 Segments)
{
	Assert(RenderBatching);
	Assert(Segments >= 3);
	if(VertexCount + Segments * 3 > MAX_VERTICES)
	{
		FlushVertexData();
	}

	f32 Theta = TAU32 / Segments;

	for(i32 Counter=0; Counter < Segments; ++Counter)
	{
		PushVertex(X, Y, ColorR, ColorG, ColorB, ColorA);
		PushVertex(X + Radius * Cos(Counter*Theta), Y + Radius * Sin(Counter*Theta), ColorR, ColorG, ColorB, ColorA);
		PushVertex(X + Radius * Cos(Counter*Theta + Theta), Y + Radius * Sin(Counter*Theta + Theta), ColorR, ColorG, ColorB, ColorA);
	}
}

void FinishRenderBatch()
{
	Assert(RenderBatching);
	RenderBatching = false;
	FlushVertexData();
}
