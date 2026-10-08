"""An interleaved position/color mesh with optional triangle indices."""

import ctypes

import numpy as np
from OpenGL import GL


class Mesh:
    def __init__(self, vertices, indices=None, primitive=GL.GL_TRIANGLES):
        vertices = np.ascontiguousarray(vertices, dtype=np.float32)
        if vertices.ndim != 2 or vertices.shape[1] != 6 or not len(vertices):
            raise ValueError("Vertices must be a nonempty array of [x, y, z, r, g, b].")
        if indices is not None:
            indices = np.ascontiguousarray(indices, dtype=np.uint32).reshape(-1)
            if not len(indices) or np.any(indices >= len(vertices)):
                raise ValueError("Indices must refer to existing vertices.")

        self.primitive = primitive
        self.vao = self.vbo = self.ebo = 0
        self.count = len(indices) if indices is not None else len(vertices)
        try:
            self.vao = GL.glGenVertexArrays(1)
            self.vbo = GL.glGenBuffers(1)
            GL.glBindVertexArray(self.vao)
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.vbo)
            GL.glBufferData(
                GL.GL_ARRAY_BUFFER, vertices.nbytes, vertices, GL.GL_STATIC_DRAW
            )
            # Six float32 values per vertex: position at byte 0, color at byte 12.
            stride = 6 * vertices.itemsize
            for location, offset in ((0, 0), (1, 3 * vertices.itemsize)):
                GL.glEnableVertexAttribArray(location)
                GL.glVertexAttribPointer(
                    location, 3, GL.GL_FLOAT, GL.GL_FALSE,
                    stride, ctypes.c_void_p(offset)
                )
            if indices is not None:
                self.ebo = GL.glGenBuffers(1)
                GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, self.ebo)
                GL.glBufferData(
                    GL.GL_ELEMENT_ARRAY_BUFFER, indices.nbytes,
                    indices, GL.GL_STATIC_DRAW
                )
        except Exception:
            self.close()
            raise
        finally:
            # The EBO binding is stored in the VAO; leave it attached.
            GL.glBindVertexArray(0)
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)

    def draw(self):
        GL.glBindVertexArray(self.vao)
        if self.ebo:
            GL.glDrawElements(
                self.primitive, self.count, GL.GL_UNSIGNED_INT, ctypes.c_void_p(0)
            )
        else:
            GL.glDrawArrays(self.primitive, 0, self.count)
        GL.glBindVertexArray(0)

    def close(self):
        if self.ebo:
            GL.glDeleteBuffers(1, [self.ebo])
            self.ebo = 0
        if self.vbo:
            GL.glDeleteBuffers(1, [self.vbo])
            self.vbo = 0
        if self.vao:
            GL.glDeleteVertexArrays(1, [self.vao])
            self.vao = 0
