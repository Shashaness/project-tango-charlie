"""Compile GLSL files and manage a shader program."""

from pathlib import Path

import numpy as np
from OpenGL import GL


class Shader:
    def __init__(self, vertex_path, fragment_path):
        self.program = 0
        shaders = []
        try:
            for path, shader_type in (
                (vertex_path, GL.GL_VERTEX_SHADER),
                (fragment_path, GL.GL_FRAGMENT_SHADER),
            ):
                source = Path(path).read_text(encoding="utf-8")
                shader = GL.glCreateShader(shader_type)
                shaders.append(shader)
                GL.glShaderSource(shader, source)
                GL.glCompileShader(shader)
                if not GL.glGetShaderiv(shader, GL.GL_COMPILE_STATUS):
                    log = GL.glGetShaderInfoLog(shader)
                    raise RuntimeError(f"Shader compilation failed ({path}):\n{log}")

            self.program = GL.glCreateProgram()
            for shader in shaders:
                GL.glAttachShader(self.program, shader)
            GL.glLinkProgram(self.program)
            if not GL.glGetProgramiv(self.program, GL.GL_LINK_STATUS):
                log = GL.glGetProgramInfoLog(self.program)
                raise RuntimeError(
                    f"Shader link failed ({vertex_path}, {fragment_path}):\n{log}"
                )
        except Exception:
            self.close()
            raise
        finally:
            for shader in shaders:
                # Linked programs retain the compiled code after shader deletion.
                if self.program:
                    GL.glDetachShader(self.program, shader)
                GL.glDeleteShader(shader)

    def use(self):
        GL.glUseProgram(self.program)

    def set_matrix(self, name, matrix):
        """Upload a mathematical row-major NumPy matrix to the active program."""
        location = GL.glGetUniformLocation(self.program, name)
        if location == -1:
            raise ValueError(f"Matrix uniform {name!r} was not found in the shader.")
        matrix = np.asarray(matrix, dtype=np.float32)
        if matrix.shape != (4, 4):
            raise ValueError("Matrix uniforms must have shape (4, 4).")
        # OpenGL expects column-major memory. Transpose the storage, not the math.
        GL.glUniformMatrix4fv(
            location, 1, GL.GL_FALSE, np.ascontiguousarray(matrix.T)
        )

    def close(self):
        if self.program:
            GL.glDeleteProgram(self.program)
            self.program = 0
