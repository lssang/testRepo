'''OpenGL extension EXT.texture_compression_rgtc

This module customises the behaviour of the 
OpenGL.raw.GL.EXT.texture_compression_rgtc to provide a more 
Python-friendly API

Overview (from the spec)
	
	This extension introduces four new block-based texture compression
	formats suited for unsigned and signed red and red-green textures
	(hence the name "rgtc" for Red-Green Texture Compression).
	
	These formats are designed to reduce the storage requirements
	and memory bandwidth required for red and red-green textures by
	a factor of 2-to-1 over conventional uncompressed luminance and
	luminance-alpha textures with 8-bit components (GL_LUMINANCE8 and
	GL_LUMINANCE8_ALPHA8).
	
	The compressed signed red-green format is reasonably suited for
	storing compressed normal maps.
	
	This extension uses the same compression format as the
	EXT_texture_compression_latc extension except the color data is stored
	in the red and green components rather than luminance and alpha.
	Representing compressed red and green components is consistent with
	the BC4 and BC5 compressed formats supported by DirectX 10.

The official definition of this extension is available here:
http://www.opengl.org/registry/specs/EXT/texture_compression_rgtc.txt
'''
from OpenGL import platform, constants, constant, arrays
from OpenGL import extensions, wrapper
from OpenGL.GL import glget
import ctypes
from OpenGL.raw.GL.EXT.texture_compression_rgtc import *
### END AUTOGENERATED SECTION