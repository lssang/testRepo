'''OpenGL extension VERSION.GL_1_2

This module customises the behaviour of the 
OpenGL.raw.GL.VERSION.GL_1_2 to provide a more 
Python-friendly API

The official definition of this extension is available here:
http://www.opengl.org/registry/specs/VERSION/GL_1_2.txt
'''
from OpenGL import platform, constants, constant, arrays
from OpenGL import extensions, wrapper
from OpenGL.GL import glget
import ctypes
from OpenGL.raw.GL.VERSION.GL_1_2 import *
### END AUTOGENERATED SECTION
from OpenGL.GL.VERSION.GL_1_2_images import *

glDrawRangeElements = wrapper.wrapper( simple.glDrawRangeElements ).setPyConverter(
    'indices', arrays.AsArrayOfType( 'indices', 'type' ),
).setReturnValues(
    wrapper.returnPyArgument( 'indices' )
)