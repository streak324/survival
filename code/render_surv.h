#ifndef RENDER_SURV_H
#define RENDER_SURV_H

#include "glad.h"
#include "common_surv.h"

GLuint CreateGLShader(const char* ShaderSource, GLenum ShaderType);
GLuint CreateVFShader(const char* VertexSource, const char* FragSource);
void StartRenderBatch();
void RenderQuad(f32 X, f32 Y, f32 Width, f32 Height, f32 Rotation, f32 ColorR, f32 ColorG, f32 ColorB, f32 ColorA);
void RenderCircle(f32 X, f32 Y, f32 Radius, f32 ColorR, f32 ColorG, f32 ColorB, f32 ColorA, i32 Segments);
void FinishRenderBatch();

#endif
