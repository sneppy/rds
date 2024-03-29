#pragma once

#include "coremin.h"

/// @brief OpenGL libraries includes
#include <GL/glcorearb.h>
#include <GL/glext.h>

/////////////////////////////////////////////////
// OpenGL functions entry points               //
/////////////////////////////////////////////////

#define GL_ENTRYPOINTS(expansion) \
	expansion(PFNGLCLEARCOLORPROC, glClearColor) \
	expansion(PFNGLCLEARPROC, glClear) \
	expansion(PFNGLFLUSHPROC, glFlush) \
	expansion(PFNGLGENVERTEXARRAYSPROC, glGenVertexArrays) \
	expansion(PFNGLBINDVERTEXARRAYPROC, glBindVertexArray) \
	expansion(PFNGLBINDFRAMEBUFFERPROC, glBindFramebuffer) \
	expansion(PFNGLENABLEPROC, glEnable) \
	expansion(PFNGLDISABLEPROC, glDisable) \
	expansion(PFNGLGENBUFFERSPROC, glGenBuffers) \
	expansion(PFNGLDELETEBUFFERSPROC, glDeleteBuffers) \
	expansion(PFNGLBINDBUFFERPROC, glBindBuffer) \
	expansion(PFNGLBUFFERDATAPROC, glBufferData) \
	expansion(PFNGLBUFFERSUBDATAPROC, glBufferSubData) \
	expansion(PFNGLNAMEDBUFFERDATAPROC, glNamedBufferData) \
	expansion(PFNGLVERTEXATTRIBPOINTERPROC, glVertexAttribPointer) \
	expansion(PFNGLENABLEVERTEXATTRIBARRAYPROC, glEnableVertexAttribArray) \
	expansion(PFNGLDRAWARRAYSPROC, glDrawArrays) \
	expansion(PFNGLDRAWELEMENTSPROC, glDrawElements) \
	expansion(PFNGLDRAWRANGEELEMENTSPROC, glDrawRangeElements) \
	expansion(PFNGLPOLYGONMODEPROC, glPolygonMode) \
	expansion(PFNGLCOMPILESHADERPROC, glCompileShader) \
	expansion(PFNGLATTACHSHADERPROC, glAttachShader) \
	expansion(PFNGLCREATEPROGRAMPROC, glCreateProgram) \
	expansion(PFNGLLINKPROGRAMPROC, glLinkProgram) \
	expansion(PFNGLUSEPROGRAMPROC, glUseProgram) \
	expansion(PFNGLCREATESHADERPROC, glCreateShader) \
	expansion(PFNGLSHADERSOURCEPROC, glShaderSource) \
	expansion(PFNGLGETUNIFORMLOCATIONPROC, glGetUniformLocation) \
	expansion(PFNGLUNIFORMMATRIX4FVPROC, glUniformMatrix4fv) \
	expansion(PFNGLREADBUFFERPROC, glReadBuffer) \
	expansion(PFNGLDRAWBUFFERPROC, glDrawBuffer) \
	expansion(PFNGLBLITFRAMEBUFFERPROC, glBlitFramebuffer) \
	expansion(PFNGLFINISHPROC, glFinish) \
	expansion(PFNGLBINDTEXTUREPROC, glBindTexture) \
	expansion(PFNGLFRONTFACEPROC, glFrontFace) \
	expansion(PFNGLGETSTRINGPROC, glGetString) \
	expansion(PFNGLGETPROGRAMIVPROC, glGetProgramiv) \
	expansion(PFNGLISSHADERPROC, glIsShader) \
	expansion(PFNGLGETSHADERIVPROC, glGetShaderiv) \
	expansion(PFNGLGETSHADERINFOLOGPROC, glGetShaderInfoLog) \

#define GL_DECLARE_ENTRYPOINTS(type, func) extern type func;

/// @brief Namespace for OpenGL functions

namespace GLFuncPointers
{
	GL_ENTRYPOINTS(GL_DECLARE_ENTRYPOINTS)
}
using namespace GLFuncPointers;

#undef GL_DECLARE_ENTRYPOINTS


